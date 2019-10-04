#ifndef __LINUX_WORKQUEUE_H__
#define __LINUX_WORKQUEUE_H__

struct work_struct { } ;

#define INIT_WORK(x,y,z) /* */
#define schedule_work(x) do { } while(0)
#define flush_scheduled_work() do { } while(0)

#define queue_delayed_work(workqueue, delayed_work, delay_ms) \
({ \
  int ret = 0; \
  rtems_status_code delayed_work_sc = rtems_timer_fire_after((delayed_work)->id, (delay_ms)*rtems_clock_get_ticks_per_second()/1000, delayed_work_service_routine, &(delayed_work)->work); \
  if (delayed_work_sc != RTEMS_SUCCESSFUL) { \
    ret = 1; \
  } \
  ret; \
})

#define INIT_DELAYED_WORK(delayed_work, delayed_workqueue_callback) \
	rtems_status_code delayed_work_sc = rtems_timer_create(rtems_build_name('T', 'M', 'D', 'W'), &(delayed_work)->id); \
	if (delayed_work_sc != RTEMS_SUCCESSFUL) { \
	  return -ENOMEM; \
	} \
	(delayed_work)->callback = delayed_workqueue_callback;


#define msecs_to_jiffies(a) (a)

typedef void (*work_callback_t)(struct work_struct *work);
struct delayed_work {
	struct work_struct work;
	rtems_id id;
	work_callback_t callback;
};

#define to_delayed_work(work) RTEMS_CONTAINER_OF(work, struct delayed_work, work)
rtems_timer_service_routine delayed_work_service_routine(
		rtems_id ignored_id,
		void *data);

#endif /* __LINUX_WORKQUEUE_H__ */
