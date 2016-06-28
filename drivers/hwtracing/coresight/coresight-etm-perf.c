/*
 * Copyright(C) 2015 Linaro Limited. All rights reserved.
 * Author: Mathieu Poirier <mathieu.poirier@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/coresight.h>
#include <linux/coresight-pmu.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/parser.h>
#include <linux/perf_event.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "coresight-priv.h"
#include "coresight-etm-perf.h"

static struct pmu etm_pmu;
static bool etm_perf_up;

/**
 * struct etm_event_data - Coresight specifics associated to an event
 * @work:		Handle to free allocated memory outside IRQ context.
 * @mask:		Hold the CPU(s) this event was set for.
 * @snk_config:		The sink configuration.
 * @path:		An array of path, each slot for one CPU.
 */
struct etm_event_data {
	struct work_struct work;
	cpumask_t mask;
	void *snk_config;
	struct list_head **path;
};

static DEFINE_PER_CPU(struct perf_output_handle, ctx_handle);
static DEFINE_PER_CPU(struct coresight_device *, csdev_src);

/* ETMv3.5/PTM's ETMCR is 'config' */
PMU_FORMAT_ATTR(cycacc,		"config:" __stringify(ETM_OPT_CYCACC));
PMU_FORMAT_ATTR(timestamp,	"config:" __stringify(ETM_OPT_TS));

static struct attribute *etm_config_formats_attr[] = {
	&format_attr_cycacc.attr,
	&format_attr_timestamp.attr,
	NULL,
};

static struct attribute_group etm_pmu_format_group = {
	.name   = "format",
	.attrs  = etm_config_formats_attr,
};

static const struct attribute_group *etm_pmu_attr_groups[] = {
	&etm_pmu_format_group,
	NULL,
};

static void etm_event_read(struct perf_event *event) {}

static int etm_addr_filters_alloc(struct perf_event *event)
{
	struct etm_filters *filters;
	int node = event->cpu == -1 ? -1 : cpu_to_node(event->cpu);

	filters = kzalloc_node(sizeof(struct etm_filters), GFP_KERNEL, node);
	if (!filters)
		return -ENOMEM;

	if (event->parent)
		memcpy(filters, event->parent->hw.addr_filters,
		       sizeof(*filters));

	event->hw.addr_filters = filters;

	return 0;
}

static void etm_event_destroy(struct perf_event *event)
{
	void *sink;

	sink = perf_event_drv_configs_set(event, NULL);
	kfree(sink);
	kfree(event->hw.addr_filters);
	event->hw.addr_filters = NULL;
}

static int etm_event_init(struct perf_event *event)
{
	struct hw_perf_event *hw_event = &event->hw;
	int ret;

	if (event->attr.type != etm_pmu.type)
		return -ENOENT;

	event->destroy = etm_event_destroy;
	raw_spin_lock_init(&hw_event->drv_configs_lock);
	hw_event->drv_configs = NULL;

	ret = etm_addr_filters_alloc(event);
	if (ret)
		return ret;

	return 0;
}

static void free_event_data(struct work_struct *work)
{
	int cpu;
	cpumask_t *mask;
	struct etm_event_data *event_data;
	struct coresight_device *sink;

	event_data = container_of(work, struct etm_event_data, work);
	mask = &event_data->mask;
	/*
	 * First deal with the sink configuration.  See comment in
	 * etm_setup_aux() about why we take the first available path.
	 */
	if (event_data->snk_config) {
		cpu = cpumask_first(mask);
		sink = coresight_get_sink(event_data->path[cpu]);
		if (sink_ops(sink)->free_buffer)
			sink_ops(sink)->free_buffer(event_data->snk_config);
	}

	for_each_cpu(cpu, mask) {
		if (event_data->path[cpu])
			coresight_release_path(event_data->path[cpu]);
	}

	kfree(event_data->path);
	kfree(event_data);
}

static void *alloc_event_data(int cpu)
{
	int size;
	cpumask_t *mask;
	struct etm_event_data *event_data;

	/* First get memory for the session's data */
	event_data = kzalloc(sizeof(struct etm_event_data), GFP_KERNEL);
	if (!event_data)
		return NULL;

	/* Make sure nothing disappears under us */
	get_online_cpus();
	size = num_online_cpus();

	mask = &event_data->mask;
	if (cpu != -1)
		cpumask_set_cpu(cpu, mask);
	else
		cpumask_copy(mask, cpu_online_mask);
	put_online_cpus();

	/*
	 * Each CPU has a single path between source and destination.  As such
	 * allocate an array using CPU numbers as indexes.  That way a path
	 * for any CPU can easily be accessed at any given time.  We proceed
	 * the same way for sessions involving a single CPU.  The cost of
	 * unused memory when dealing with single CPU trace scenarios is small
	 * compared to the cost of searching through an optimized array.
	 */
	event_data->path = kcalloc(size,
				   sizeof(struct list_head *), GFP_KERNEL);
	if (!event_data->path) {
		kfree(event_data);
		return NULL;
	}

	return event_data;
}

static void etm_free_aux(void *data)
{
	struct etm_event_data *event_data = data;

	schedule_work(&event_data->work);
}

static void *etm_setup_aux(struct perf_event *event, void **pages,
			   int nr_pages, bool overwrite)
{
	int cpu;
	char *cmdl_sink;
	cpumask_t *mask;
	struct coresight_device *sink;
	struct etm_event_data *event_data = NULL;

	event_data = alloc_event_data(event->cpu);
	if (!event_data)
		return NULL;

	INIT_WORK(&event_data->work, free_event_data);

	mask = &event_data->mask;

	/*
	 * If a sink was specified from the perf cmdline it will be part of
	 * the event's drv_configs.
	 */
	cmdl_sink = (char *)perf_event_drv_configs_get(event);
	if (IS_ERR(cmdl_sink))
		cmdl_sink = NULL;

	/* Setup the path for each CPU in a trace session */
	for_each_cpu(cpu, mask) {
		struct coresight_device *csdev;

		csdev = per_cpu(csdev_src, cpu);
		if (!csdev)
			goto err;

		/*
		 * Building a path doesn't enable it, it simply builds a
		 * list of devices from source to sink that can be
		 * referenced later when the path is actually needed.
		 */
		event_data->path[cpu] = coresight_build_path(csdev, cmdl_sink);
		if (!event_data->path[cpu])
			goto err;
	}

	/*
	 * In theory nothing prevent tracers in a trace session from being
	 * associated with different sinks, nor having a sink per tracer.  But
	 * until we have HW with this kind of topology and a way to convey
	 * sink assignement from the perf cmd line we need to assume tracers
	 * in a trace session are using the same sink.  Therefore pick the sink
	 * found at the end of the first available path.
	 */
	cpu = cpumask_first(mask);
	/* Grab the sink at the end of the path */
	sink = coresight_get_sink(event_data->path[cpu]);
	if (!sink)
		goto err;

	if (!sink_ops(sink)->alloc_buffer)
		goto err;

	/* Get the AUX specific data from the sink buffer */
	event_data->snk_config =
			sink_ops(sink)->alloc_buffer(sink, cpu, pages,
						     nr_pages, overwrite);
	if (!event_data->snk_config)
		goto err;

out:
	return event_data;

err:
	etm_free_aux(event_data);
	event_data = NULL;
	goto out;
}

static void etm_event_start(struct perf_event *event, int flags)
{
	int cpu = smp_processor_id();
	struct etm_event_data *event_data;
	struct perf_output_handle *handle = this_cpu_ptr(&ctx_handle);
	struct coresight_device *sink, *csdev = per_cpu(csdev_src, cpu);

	if (!csdev)
		goto fail;

	/*
	 * Deal with the ring buffer API and get a handle on the
	 * session's information.
	 */
	event_data = perf_aux_output_begin(handle, event);
	if (!event_data)
		goto fail;

	/* We need a sink, no need to continue without one */
	sink = coresight_get_sink(event_data->path[cpu]);
	if (WARN_ON_ONCE(!sink || !sink_ops(sink)->set_buffer))
		goto fail_end_stop;

	/* Configure the sink */
	if (sink_ops(sink)->set_buffer(sink, handle,
				       event_data->snk_config))
		goto fail_end_stop;

	/* Nothing will happen without a path */
	if (coresight_enable_path(event_data->path[cpu], CS_MODE_PERF))
		goto fail_end_stop;

	/* Tell the perf core the event is alive */
	event->hw.state = 0;

	/* Finally enable the tracer */
	if (source_ops(csdev)->enable(csdev, event, CS_MODE_PERF))
		goto fail_end_stop;

out:
	return;

fail_end_stop:
	perf_aux_output_end(handle, 0, true);
fail:
	event->hw.state = PERF_HES_STOPPED;
	goto out;
}

static void etm_event_stop(struct perf_event *event, int mode)
{
	bool lost;
	int cpu = smp_processor_id();
	unsigned long size;
	struct coresight_device *sink, *csdev = per_cpu(csdev_src, cpu);
	struct perf_output_handle *handle = this_cpu_ptr(&ctx_handle);
	struct etm_event_data *event_data = perf_get_aux(handle);

	if (event->hw.state == PERF_HES_STOPPED)
		return;

	if (!csdev)
		return;

	sink = coresight_get_sink(event_data->path[cpu]);
	if (!sink)
		return;

	/* stop tracer */
	source_ops(csdev)->disable(csdev, event);

	/* tell the core */
	event->hw.state = PERF_HES_STOPPED;

	if (mode & PERF_EF_UPDATE) {
		if (WARN_ON_ONCE(handle->event != event))
			return;

		/* update trace information */
		if (!sink_ops(sink)->update_buffer)
			return;

		sink_ops(sink)->update_buffer(sink, handle,
					      event_data->snk_config);

		if (!sink_ops(sink)->reset_buffer)
			return;

		size = sink_ops(sink)->reset_buffer(sink, handle,
						    event_data->snk_config,
						    &lost);

		perf_aux_output_end(handle, size, lost);
	}

	/* Disabling the path make its elements available to other sessions */
	coresight_disable_path(event_data->path[cpu]);
}

static int etm_event_add(struct perf_event *event, int mode)
{
	int ret = 0;
	struct hw_perf_event *hwc = &event->hw;

	if (mode & PERF_EF_START) {
		etm_event_start(event, 0);
		if (hwc->state & PERF_HES_STOPPED)
			ret = -EINVAL;
	} else {
		hwc->state = PERF_HES_STOPPED;
	}

	return ret;
}

static void etm_event_del(struct perf_event *event, int mode)
{
	etm_event_stop(event, PERF_EF_UPDATE);
}

enum {
	ETM_TOKEN_SINK_CPU,
	ETM_TOKEN_SINK,
	ETM_TOKEN_ERR,
};

static const match_table_t drv_cfg_tokens = {
	{ETM_TOKEN_SINK_CPU, "sink=cpu%d:%s"},
	{ETM_TOKEN_SINK, "sink=%s"},
	{ETM_TOKEN_ERR, NULL},
};

/**
 * etm_parse_drv_configs - parse configuration string
 *
 * Returns: ERR_PTR() upon error,  NULL if the config string CPU is different
 *	    from the event's CPU and the address of a valid string if one has
 *	    been found.
 */
static char *etm_parse_drv_configs(int event_cpu, char *config_str)
{
	char *sink = NULL, *config = config_str;
	int cpu, token;
	substring_t args[MAX_OPT_ARGS];

	/* See above declared @drv_cfg_tokens for the usable formats */
	token = match_token(config, drv_cfg_tokens, args);
	switch (token) {
	case ETM_TOKEN_SINK:
		/* Simply return the sink that was specified */
		sink = match_strdup(&args[0]);
		break;
	case ETM_TOKEN_SINK_CPU:
		/*
		 * We have a sink and a CPU, so we need to make sure the
		 * given CPU matches the event's CPU.  If not this sink is
		 * destined to another tracer.  This isn't an error hence not
		 * setting @ret.
		 */

		/* Get the cpu */
		if (match_int(&args[0], &cpu)) {
			sink = ERR_PTR(-EINVAL);
			goto out;
		}

		/* This sink configuration is meant for another CPU */
		if (event_cpu != cpu)
			goto out;

		sink = match_strdup(&args[1]);
		break;
	default:
		sink = ERR_PTR(-EINVAL);
	}

out:
	return sink;
}

static int
etm_set_drv_configs(struct perf_event *event,
		    struct list_head *drv_configs)
{
	char *config, *sink;
	int len;
	struct perf_drv_config *drv_config;
	void *old_sink;

	list_for_each_entry(drv_config, drv_configs, entry) {
		/* ETM HW configuration needs a sink specification */
		if (!drv_config->option)
			return -EINVAL;

		len = strlen(drv_config->config) + strlen("=") +
		      strlen(drv_config->option) + 1;

		config = kmalloc(len, GFP_KERNEL);
		if (!config)
			return -ENOMEM;

		/* Reconstruct user configuration */
		snprintf(config, len, "%s=%s",
			 drv_config->config, drv_config->option);

		sink = etm_parse_drv_configs(event->cpu, config);
		kfree(config);

		/* No need to continue if the parse encountered and error */
		if (IS_ERR(sink))
			return PTR_ERR(sink);

		/* A valid sink was found */
		if (sink)
			goto found;
	}

	/*
	 * A sink wasn't found, which isn't automatically an error.  Other
	 * options on the cmd line may still need to be parsed.
	 */
	return 0;

found:
	/* Record the sink that was found */
	old_sink = perf_event_drv_configs_set(event, sink);
	/* And get rid of whatever we had before */
	kfree(old_sink);

	return 0;
}

static int etm_addr_filters_validate(struct list_head *filters)
{
	bool range = false, address = false;
	int index = 0;
	struct perf_addr_filter *filter;

	list_for_each_entry(filter, filters, entry) {
		/*
		 * No need to go further if there's no more
		 * room for filters.
		 */
		if (++index > ETM_ADDR_CMP_MAX)
			return -EOPNOTSUPP;

		/*
		 * As taken from the struct perf_addr_filter documentation:
		 *	@range:	1: range, 0: address
		 *
		 * At this time we don't allow range and start/stop filtering
		 * to cohabitate, they have to be mutually exclusive.
		 */
		if ((filter->range == 1) && address)
			return -EOPNOTSUPP;

		if ((filter->range == 0) && range)
			return -EOPNOTSUPP;

		/*
		 * For range filtering, the second address in the address
		 * range comparator needs to be higher than the first.
		 * Invalid otherwise.
		 */
		if (filter->range && filter->size == 0)
			return -EINVAL;

		/*
		 * Everything checks out with this filter, record what we've
		 * received before moving on to the next one.
		 */
		if (filter->range)
			range = true;
		else
			address = true;
	}

	return 0;
}

static void etm_addr_filters_sync(struct perf_event *event)
{
	struct perf_addr_filters_head *head = perf_event_addr_filters(event);
	unsigned long start, stop, *offs = event->addr_filters_offs;
	struct etm_filters *filters = event->hw.addr_filters;
	struct etm_filter *etm_filter;
	struct perf_addr_filter *filter;
	int i = 0;

	list_for_each_entry(filter, &head->list, entry) {
		start = filter->offset + offs[i];
		stop = start + filter->size;
		etm_filter = &filters->etm_filter[i];

		if (filter->range == 1) {
			etm_filter->start_addr = start;
			etm_filter->stop_addr = stop;
			etm_filter->type = ETM_ADDR_TYPE_RANGE;
		} else {
			if (filter->filter == 1) {
				etm_filter->start_addr = start;
				etm_filter->type = ETM_ADDR_TYPE_START;
			} else {
				etm_filter->stop_addr = stop;
				etm_filter->type = ETM_ADDR_TYPE_STOP;
			}
		}
		i++;
	}

	filters->nr_filters = i;
}

int etm_perf_symlink(struct coresight_device *csdev, bool link)
{
	char entry[sizeof("cpu9999999")];
	int ret = 0, cpu = source_ops(csdev)->cpu_id(csdev);
	struct device *pmu_dev = etm_pmu.dev;
	struct device *cs_dev = &csdev->dev;

	sprintf(entry, "cpu%d", cpu);

	if (!etm_perf_up)
		return -EPROBE_DEFER;

	if (link) {
		ret = sysfs_create_link(&pmu_dev->kobj, &cs_dev->kobj, entry);
		if (ret)
			return ret;
		per_cpu(csdev_src, cpu) = csdev;
	} else {
		sysfs_remove_link(&pmu_dev->kobj, entry);
		per_cpu(csdev_src, cpu) = NULL;
	}

	return 0;
}

static int __init etm_perf_init(void)
{
	int ret;

	etm_pmu.capabilities		= PERF_PMU_CAP_EXCLUSIVE;

	etm_pmu.attr_groups		= etm_pmu_attr_groups;
	etm_pmu.task_ctx_nr		= perf_sw_context;
	etm_pmu.read			= etm_event_read;
	etm_pmu.event_init		= etm_event_init;
	etm_pmu.setup_aux		= etm_setup_aux;
	etm_pmu.free_aux		= etm_free_aux;
	etm_pmu.start			= etm_event_start;
	etm_pmu.stop			= etm_event_stop;
	etm_pmu.add			= etm_event_add;
	etm_pmu.del			= etm_event_del;
	etm_pmu.set_drv_configs		= etm_set_drv_configs;
	etm_pmu.addr_filters_sync	= etm_addr_filters_sync;
	etm_pmu.addr_filters_validate	= etm_addr_filters_validate;
	etm_pmu.nr_addr_filters		= ETM_ADDR_CMP_MAX;

	ret = perf_pmu_register(&etm_pmu, CORESIGHT_ETM_PMU_NAME, -1);
	if (ret == 0)
		etm_perf_up = true;

	return ret;
}
device_initcall(etm_perf_init);
