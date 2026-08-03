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

static int sysoff_estimate(struct ptp_clock_time *pct, int extended,
			   int n_samples, int64_t *result, uint64_t *ts,
			   int64_t *delay)
{
	int64_t t1, t2, tp;
	int64_t interval, timestamp, offset;
	int64_t shortest_interval = INT64_MAX;
	int64_t best_timestamp = 0, best_offset = 0;
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

static int sysoff_extended(int fd, clockid_t sys_clock, int n_samples,
			   int64_t *result, uint64_t *ts, int64_t *delay)
{
	struct ptp_sys_offset_extended pso;
	memset(&pso, 0, sizeof(pso));
	pso.n_samples = n_samples;
#ifdef HAVE_PTP_SYSOFF_EXTENDED_CLOCKID
	pso.clockid = sys_clock;
#else
	pso.rsv[0] = sys_clock;
#endif
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

int sysoff_measure(int fd, clockid_t sys_clock, int method, int n_samples,
		   int64_t *result, uint64_t *ts, int64_t *delay)
{
	switch (method) {
	case SYSOFF_PRECISE:
		if (sys_clock != CLOCK_REALTIME)
			return -EOPNOTSUPP;
		*delay = 0;
		return sysoff_precise(fd, result, ts);
	case SYSOFF_EXTENDED:
		return sysoff_extended(fd, sys_clock, n_samples,
				       result, ts, delay);
	case SYSOFF_BASIC:
		if (sys_clock != CLOCK_REALTIME)
			return -EOPNOTSUPP;
		return sysoff_basic(fd, n_samples, result, ts, delay);
	}
	return -EOPNOTSUPP;
}

int sysoff_measure_retry(int fd, clockid_t sys_clock, int method, int n_samples,
			 int n_tries, int64_t *result, uint64_t *ts,
			 int64_t *delay)
{
	int i, err = -EINVAL;

	for (i = 0; i < n_tries; i++) {
		err = sysoff_measure(fd, sys_clock, method, n_samples,
				     result, ts, delay);
		if (err != -EBUSY)
			return err;
	}

	return err;
}

int sysoff_probe(int fd, clockid_t sys_clock, int n_samples)
{
	int64_t junk, delay;
	int method, err;
	int n_tries = 3;
	uint64_t ts;

	if (n_samples > PTP_MAX_SAMPLES) {
		fprintf(stderr, "warning: %d exceeds kernel max readings %d\n",
			n_samples, PTP_MAX_SAMPLES);
		fprintf(stderr, "falling back to clock_gettime method\n");
		return SYSOFF_RUN_TIME_MISSING;
	}

	for (method = 0; method < SYSOFF_LAST; method++) {
		err = sysoff_measure_retry(fd, sys_clock, method,
					   n_samples, n_tries,
					   &junk, &ts, &delay);
		if (err)
			continue;

		return method;
	}

	return SYSOFF_RUN_TIME_MISSING;
}
