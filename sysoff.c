/**
 * @file sysoff.c
 * @brief Implements the system offset estimation method.
 * @note Copyright (C) 2012 Richard Cochran <richardcochran@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/ptp_clock.h>

#include "print.h"
#include "sysoff.h"

#define NS_PER_SEC 1000000000LL

static void print_ioctl_error(const char *name)
{
	if (errno == EOPNOTSUPP)
		pr_debug("ioctl %s: %s", name, strerror(errno));
	else
		pr_err("ioctl %s: %s", name, strerror(errno));
}

static int64_t pctns(struct ptp_clock_time *t)
{
	return t->sec * NS_PER_SEC + t->nsec;
}

static int sysoff_precise(int fd, int64_t *result, uint64_t *ts)
{
	struct ptp_sys_offset_precise pso;
	memset(&pso, 0, sizeof(pso));
	if (ioctl(fd, PTP_SYS_OFFSET_PRECISE, &pso)) {
		print_ioctl_error("PTP_SYS_OFFSET_PRECISE");
		return -errno;
	}
	*result = pctns(&pso.sys_realtime) - pctns(&pso.device);
	*ts = pctns(&pso.sys_realtime);
	return 0;
}

static int sysoff_estimate(struct ptp_clock_time *pct, int extended, int n_samples,
			   int64_t *result, uint64_t *ts, int64_t *delay)
{
	int64_t t1, t2, tp;
	int64_t interval, timestamp, offset;
	int64_t shortest_interval = INT64_MAX, best_timestamp = 0, best_offset = 0;
	int i;

	for (i = 0; i < n_samples; i++) {
		if (extended) {
			t1 = pctns(&pct[3*i]);
			tp = pctns(&pct[3*i+1]);
			t2 = pctns(&pct[3*i+2]);
		} else {
			t1 = pctns(&pct[2*i]);
			tp = pctns(&pct[2*i+1]);
			t2 = pctns(&pct[2*i+2]);
		}

		interval = t2 - t1;
		if (interval < 0)
			continue;

		timestamp = (t2 + t1) / 2;
		offset = timestamp - tp;
		if (interval < shortest_interval) {
			shortest_interval = interval;
			best_timestamp = timestamp;
			best_offset = offset;
		}
	}

	if (shortest_interval == INT64_MAX)
		return -EBUSY;

	*result = best_offset;
	*ts = best_timestamp;
	*delay = shortest_interval;
	return 0;
}

static int sysoff_extended(int fd, int n_samples,
			   int64_t *result, uint64_t *ts, int64_t *delay)
{
	struct ptp_sys_offset_extended pso;
	memset(&pso, 0, sizeof(pso));
	pso.n_samples = n_samples;
	if (ioctl(fd, PTP_SYS_OFFSET_EXTENDED, &pso)) {
		print_ioctl_error("PTP_SYS_OFFSET_EXTENDED");
		return -errno;
	}
	return sysoff_estimate(&pso.ts[0][0], 1, n_samples, result, ts, delay);
}

static int sysoff_basic(int fd, int n_samples,
			int64_t *result, uint64_t *ts, int64_t *delay)
{
	struct ptp_sys_offset pso;
	memset(&pso, 0, sizeof(pso));
	pso.n_samples = n_samples;
	if (ioctl(fd, PTP_SYS_OFFSET, &pso)) {
		print_ioctl_error("PTP_SYS_OFFSET");
		return -errno;
	}
	return sysoff_estimate(pso.ts, 0, n_samples, result, ts, delay);
}

int sysoff_measure(int fd, int method, int n_samples,
		   int64_t *result, uint64_t *ts, int64_t *delay)
{
	switch (method) {
	case SYSOFF_PRECISE:
		*delay = 0;
		return sysoff_precise(fd, result, ts);
	case SYSOFF_EXTENDED:
		return sysoff_extended(fd, n_samples, result, ts, delay);
	case SYSOFF_BASIC:
		return sysoff_basic(fd, n_samples, result, ts, delay);
	}
	return -EOPNOTSUPP;
}

int sysoff_measure2(int fd1, int method1, int fd2, int method2, int n_samples,
		    int64_t *result, uint64_t *ts, int64_t *delay)
{
	struct timespec first, last;
	int64_t interval, result1, result2, result3;
	int64_t delay1, delay2, delay3, drift;
	uint64_t ts1, ts2, ts3;
	int err;

	clock_gettime(CLOCK_MONOTONIC, &first);

	err = sysoff_measure(fd1, method1, n_samples,
			     &result1, &ts1, &delay1);
	if (err)
		return err;

	err = sysoff_measure(fd2, method2, n_samples,
			     &result2, &ts2, &delay2);
	if (err)
		return err;

	err = sysoff_measure(fd1, method1, n_samples,
			     &result3, &ts3, &delay3);
	if (err)
		return err;

	clock_gettime(CLOCK_MONOTONIC, &last);

	/* validate the interval: CLOCK_REALTIME in sysoff may have stepped */
	interval = (last.tv_sec - first.tv_sec) * NS_PER_SEC +
		last.tv_nsec - first.tv_nsec;
	if (interval < 0 || ts3 < ts1 || ts3 - ts1 > interval)
		return -EBUSY;

	/* estimate the accumulated drift, to be added to delay */
	drift = result3 - result1;
	if (drift < 0)
		drift = -drift;

	/* use the maximum of the first or third delay */
	if (delay1 < delay3)
		delay1 = delay3;

	/* average the offsets of the first clock */
	result1 = (result1 + result3) / 2;

	/* get PHC-to-PHC offset by subtracting sys offsets */
	*result = result1 - result2;
	/* use the second PHC's time, not system time for ts */
	*ts = ts2 - result2;
	/* sum of the delays + drift */
	*delay = delay1 + delay2 + drift;

	return 0;
}

int sysoff_probe(int fd, int n_samples)
{
	int64_t junk, delay;
	int i, j, err;
	uint64_t ts;

	if (n_samples > PTP_MAX_SAMPLES) {
		fprintf(stderr, "warning: %d exceeds kernel max readings %d\n",
			n_samples, PTP_MAX_SAMPLES);
		fprintf(stderr, "falling back to clock_gettime method\n");
		return SYSOFF_RUN_TIME_MISSING;
	}

	for (i = 0; i < SYSOFF_LAST; i++) {
		for (j = 0; j < 3; j++) {
			err = sysoff_measure(fd, i, n_samples, &junk, &ts,
					     &delay);
			if (err == -EBUSY)
				continue;
			if (err)
				break;
			return i;
		}
	}

	return SYSOFF_RUN_TIME_MISSING;
}
