# DUOS Task Scheduling and Syscall Implementation

## Overview
This implementation adds comprehensive task scheduling and system call support to DUOS (Dhaka University Operating System) running on STM32F446RE (ARM Cortex-M4).

## Implemented Features

### 1. Task Control Block (TCB) - Complete Structure
- **File**: `src/kern/include/kern/types.h`
- **Structure**: `TCB_TypeDef` with all required fields:
  - `magic_number`: 0xFECABAA0
  - `task_id`: Unique ID starting from 1000
  - `psp`: Process Stack Pointer
  - `status`: Task state (NEW, READY, RUNNING, BLOCKED, TERMINATED)
  - `priority`: Task priority level
  - `parent_id`: Parent task ID
  - `execution_time`: Total execution time in ms
  - `waiting_time`: Total waiting time in ms
  - `digital_signature`: 0x00000001
  - `w_chld[16]`: Array of child task IDs
  - `heap_mem_start`: Heap memory pointer
  - `heap_mem_size`: Heap size
  - `open_resources[8]`: Open file descriptors
  - `sem_waiting_count`: Semaphore counter
  - `mutex_locked_count`: Mutex counter
  - `last_wakeup_time`: For sleep operations

### 2. Task Scheduler (Round-Robin)
- **Files**: 
  - `src/kern/include/kern/schedule.h`
  - `src/kern/thread/thread.c`
- **Features**:
  - Round-robin scheduling with 10ms time slices
  - Up to 8 concurrent tasks (configurable via MAX_TASKS)
  - 1KB stack per task
  - Task creation and management functions
  - Idle task for when no tasks are ready

### 3. Context Switching
- **SysTick Handler** (`src/kern/arch/cm4/cm4.c`):
  - Triggers every 1ms
  - Invokes PendSV every 10ms for task switching
  
- **PendSV Handler** (`src/kern/arch/stm32f446re/sys_lib/stm32_startup.c`):
  - Naked function with inline assembly
  - Saves current task context (R4-R11)
  - Calls scheduler to select next task
  - Restores next task context
  - Uses PSP (Process Stack Pointer) for task stacks

### 4. System Calls (Syscalls)

#### SYS_read (ID: 50)
- **Library Function**: `read(fd, buf, size)`
- **Kernel Function**: `sys_read(fd, buf, size)`
- Reads from STDIN_FILENO using USART
- Maximum 256 bytes per read
- Terminates on '\n' or '\r'

#### SYS_write (ID: 55)
- **Library Function**: `write(fd, buf, size)`
- **Kernel Function**: `sys_write(fd, buf, size)`
- Writes to STDOUT_FILENO or STDERR_FILENO using USART
- Used by duprintf() internally

#### SYS_getpid (ID: 5)
- **Library Function**: `getpid()`
- **Kernel Function**: `sys_getpid()`
- Returns current task ID from TCB

#### SYS_time (ID: 113)
- **Library Function**: `getSysTickTime()`
- **Kernel Function**: `sys_time()`
- Returns elapsed time in milliseconds from SysTick counter

#### SYS_exit (ID: 3)
- **Library Function**: `exit_task()`
- **Kernel Function**: `sys_exit()`
- Terminates current task
- Sets status to TERMINATED
- Triggers PendSV for rescheduling

#### SYS_yield (ID: 120)
- **Library Function**: `yield()`
- **Kernel Function**: `sys_yield()`
- Voluntarily yields CPU to next task
- Triggers PendSV immediately

#### SYS_reboot (ID: 119)
- **Library Function**: `reboot()`
- **Kernel Function**: `sys_reboot()`
- Performs software reset via SCB->AIRCR
- Uses key 0x5FA and SYSRESETREQ bit

### 5. SVC (Supervisor Call) Handler
- **File**: `src/kern/arch/stm32f446re/sys_lib/stm32_startup.c`
- Determines active stack (MSP or PSP)
- Extracts syscall number and arguments from stack
- Dispatches to `syscall()` function
- Returns result via R0

### 6. Renamed Functions
- `kprintf()` → `duprintf()` (DUOS printf)
- `kscanf()` → `duscanf()` (DUOS scanf)
- Legacy aliases maintained for compatibility

## File Structure

```
src/kern/
├── include/
│   ├── kern/
│   │   ├── types.h          (Updated TCB structure)
│   │   ├── schedule.h       (New - scheduler API)
│   │   ├── kunistd.h        (Updated - syscall prototypes)
│   │   └── syscall_def.h    (Syscall IDs)
│   └── kstdio.h            (Updated - duprintf/duscanf)
├── thread/
│   └── thread.c            (New - scheduler implementation)
├── syscall/
│   └── syscall.c           (Updated - syscall dispatcher)
├── lib/
│   ├── kunistd.c           (Updated - syscall implementations)
│   └── kstdio.c            (Updated - renamed functions)
├── arch/
│   ├── cm4/
│   │   └── cm4.c           (Updated - SysTick handler)
│   └── stm32f446re/
│       └── sys_lib/
│           └── stm32_startup.c  (Updated - SVC and PendSV handlers)
└── kmain/
    └── kmain.c             (Updated - test application)
```

## Test Application

The `kmain.c` file demonstrates:
1. **Task 1**: Prints PID and time, tests syscalls
2. **Task 2**: Tests string I/O with duscanf()
3. **Task 3**: Tests counter and voluntary yield

All tasks run concurrently with 10ms time slices.

## How It Works

### Initialization Sequence
1. `kmain()` calls `__sys_init()` to initialize hardware
2. `scheduler_init()` initializes task structures and creates idle task
3. Multiple tasks created with `task_create()`
4. PSP set to first task's stack
5. Processor switched to unprivileged mode using PSP
6. SysTick starts triggering every 1ms

### Context Switch Flow
1. SysTick interrupt every 1ms
2. Every 10th tick, trigger PendSV
3. PendSV handler:
   - Save R4-R11 to current task stack
   - Update current_task->psp
   - Call `schedule()` to select next task
   - Restore R4-R11 from next task stack
   - Update PSP
   - Return (hardware restores R0-R3, R12, LR, PC, xPSR)

### Syscall Flow
1. User code calls library function (e.g., `getpid()`)
2. Library function executes SVC instruction with syscall ID
3. SVC_Handler:
   - Determines stack (MSP/PSP)
   - Extracts arguments from stack frame
   - Calls `syscall()` dispatcher
4. Dispatcher calls kernel function (e.g., `sys_getpid()`)
5. Return value placed back in stack frame
6. Return to user code

## EXC_RETURN Values

- **0xFFFFFFFD**: Return to Thread mode using PSP (used for task switching)
- **0xFFFFFFF9**: Return to Thread mode using MSP
- **0xFFFFFFF1**: Return to Handler mode using MSP

## Priority Levels

- **SysTick**: Default priority
- **PendSV**: Lowest priority (15) - ensures clean context switching
- **SVC**: Higher priority than PendSV

## Testing

### Basic Syscall Test
```c
int32_t pid = getpid();           // Get task ID
uint32_t time = getSysTickTime(); // Get elapsed time
duprintf("PID: %d, Time: %d\r\n", pid, time);
```

### Task Yield Test
```c
yield();  // Voluntarily give CPU to next task
```

### Task Exit Test
```c
exit_task();  // Terminate current task
```

### Reboot Test
```c
reboot();  // Software reset
```

## Notes

1. **Privileged vs Unprivileged**: Tasks run in unprivileged mode, syscalls execute in privileged mode
2. **Stack Management**: Each task has separate PSP, kernel uses MSP
3. **Atomic Operations**: Context switch is atomic (interrupts disabled during critical section)
4. **No Busy-Wait**: Idle task uses WFI (Wait For Interrupt) to save power
5. **Error Handling**: All syscalls return appropriate error codes (errno.h)

## Future Enhancements

Additional syscalls that can be implemented:
- SYS_fork: Process creation
- SYS_wait: Parent waits for child
- SYS_sleep: Timed blocking
- SYS_open/close: File operations
- SYS_sem_wait/post: Semaphores
- SYS_mutex_lock/unlock: Mutexes
- SYS_signal: Signal handling

## Compilation

Use the provided Makefile in `src/compile/`:
```bash
cd src/compile
make clean
make
```

The binary will be in `src/compile/target/duos`.

## References

- ARM Cortex-M4 Technical Reference Manual
- STM32F446xx Reference Manual
- Assignment instructions (assignment_02.txt)
