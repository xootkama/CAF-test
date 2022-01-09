#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/seqlock.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/file.h>
#include <linux/string.h>
#include <linux/psi.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/list_sort.h>
#include <linux/oom.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/gfp.h>
#include <linux/swap.h>
#include <linux/timex.h>
#include <linux/jiffies.h>
#include <linux/cpuset.h>
#include <linux/export.h>
#include <linux/notifier.h>
#include <linux/memcontrol.h>
#include <linux/mempolicy.h>
#include <linux/security.h>
#include <linux/ptrace.h>
#include <linux/freezer.h>
#include <linux/ftrace.h>
#include <linux/ratelimit.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/mmu_notifier.h>
#include <asm/tlb.h>
#include <linux/psi.h>
#include <linux/oom.h>
#include "olmk.h"

#define PSI_THRESHOLD_MAX_LEN 128
static DEFINE_MUTEX(trigger_mutex);
static void	 *trigger_ptr;


static char psi_thresholds[][PSI_THRESHOLD_MAX_LEN] = {
	{ "some 100000 1000000"},   /* 100ms out of 1s for partial stall */
	{ "full 70000  1000000"},     /* 70ms out of 1s for complete stall */
};
struct work_struct	psi_lmk_work;

struct task_points {
    char comm[TASK_COMM_LEN];
    int pid;
    int adj;
    int tasksize;
    int tasksize_swap;
    unsigned long points;
    struct list_head list;
};
static struct task_points tskp_all[150];

/*
 * [0]:100->0, total_ram * 2 / ([val] * 5)
 */
static unsigned int tasksize_mb_to_adj[10] = {2000, 300, 300, 300, 300, 200, 200, 160, 160, 160};
static int adj_index = 10;

int cache_app_gap =200;


extern int psi_mem_notifier_register(struct notifier_block *nb);
static int psi_level_adj_size = 3;
int psi_level;
enum psi_mem_level {
	PSI_MEM_LEVEL_LOW = 0,
	PSI_MEM_LEVEL_MEDIUM ,
	PSI_MEM_LEVEL_CRITICAL,
	PSI_MEM_LEVEL_COUNT
};
static unsigned int psi_level_adj[PSI_MEM_LEVEL_COUNT] = {
	OOM_SCORE_ADJ_MAX + 1,
	800,
	250,
};
module_param_array_named(psi_level_adj, psi_level_adj, uint, &psi_level_adj_size,
			 S_IRUGO | S_IWUSR);

int psi_adjust_minadj(short *min_score_adj)
{

    *min_score_adj = psi_level_adj[psi_level];

    return 0;
}
EXPORT_SYMBOL_GPL(psi_adjust_minadj);

int points_adjust_minadj(short *min_score_adj)
{
    if (*min_score_adj < OOM_SCORE_ADJ_MAX + 1) {
        if (*min_score_adj >= CACHED_APP_MIN_ADJ)
            *min_score_adj = SERVICE_B_ADJ;  

        if (*min_score_adj <= BACKUP_APP_ADJ)
            *min_score_adj = FOREGROUND_APP_ADJ;
    }

	return 0;

}
EXPORT_SYMBOL_GPL(points_adjust_minadj);

static bool MemAvailable_enough(void)
{
	int other_free, other_file;

	other_file = global_node_page_state(NR_FILE_PAGES) -
			global_node_page_state(NR_SHMEM) -
			total_swapcache_pages();
	other_free = global_page_state(NR_FREE_PAGES);


	return true;
}
static int lmk_psi_mem_monitor_notifier(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct psi_trigger *trigger =( struct psi_trigger *)data;


    if (MemAvailable_enough())
		return 0;
	psi_level = (trigger->state == PSI_MEM_SOME )? PSI_MEM_LEVEL_MEDIUM : PSI_MEM_LEVEL_CRITICAL;


	schedule_work(&psi_lmk_work);

	return 0;
}

static struct notifier_block psi_mem_monitor_nb = {
	.notifier_call = lmk_psi_mem_monitor_notifier,
};
static void psi_lmk_run_work(struct work_struct *work)
{
}

static ssize_t psi_monitor_write( const char *buf,
				 size_t nbytes, enum psi_res res)
{
	size_t buf_size;
	char buffer[PSI_THRESHOLD_MAX_LEN];
	struct psi_trigger *new;
	printk(KERN_ERR "===>PSIDEBUG:psi_monitor_write\n");
	if (static_branch_likely(&psi_disabled))
		return -EOPNOTSUPP;

	buf_size = min(nbytes, (sizeof(buffer) - 1));
	memcpy(buffer, buf, buf_size);
	buffer[buf_size - 1] = '\0';

	new = psi_trigger_create(&psi_system, buffer, nbytes, res);
	if (IS_ERR(new))
		return PTR_ERR(new);

	mutex_lock(&trigger_mutex);
	psi_trigger_replace(&trigger_ptr, new);
	mutex_unlock(&trigger_mutex);
	/*add refcount for kernel psi poll*/
	kref_get(&new->refcount);

	return nbytes;
}

static int list_do_fork_count_cmp(void *priv, struct list_head *a, struct list_head *b)
{
    struct task_points *tskp1 = NULL, *tskp2 = NULL;

    tskp1 = list_entry(a, struct task_points, list);
    tskp2 = list_entry(b, struct task_points, list);

    if (tskp1->points < tskp2->points) {
        return 1;
    }

    if (tskp1->points > tskp2->points) {
        return -1;
    }

    return 0;
}

static const struct file_operations proc_task_points_fops = {

    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
}
