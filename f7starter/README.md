# STM32 IAR starter projects
Sasha Zbrozek

## Motivation
I made this for the Stanford Solar Car Project to assist getting started with
the STM32 series of processors. The intent is to have a C/C++ project that can
compile to a binary that runs on real hardware and which showcases the structure
of a simple project that can be easily extended to handle common realtime tasks.

Hopefully this is an easy-to-use starter project that is familiar across STM32
families that will assist the team (and others!) get started in a simple
embedded environment.

## Forking
* Copy the project into your own folder.
* Rename the project (replace-in-files "f7starter" with another name).
* Update the compiler target to your specific processor.
* Update the project-wide preprocessor `#defines` for your chip and clocks.
* Update the debugger settings to match your debugging hardware.

## Major decisions
Any project is shaped by a core set of decisions; this one is no exception.

### Concurrency is handled by FreeRTOS.
Sure it's possible to write non-blocking, state-machine-driven code that can run
in a single enormous for(;;){} loop. And it's probably even a good idea in many
environments. But this is a project intended for small, time-pressured teams of
undergrads interfacing with a well-defined and static set of hardware resources.
Small chunks of well-contained code are easier to reason through and review.

### Systick timer and interrupt configuration and handling is done by FreeRTOS.
While the ST HAL can do this, it's messy and inscrutable. FreeRTOS does a fine
job of it, so I've elected to let it do so without interference.

### Full dynamic memory allocation is available but should be rarely used.
Most projects shouldn't use runtime memory allocation. It's an easy way to leak
memory and cause difficult-to-hunt crashes. But it's also an enabling component
of some high-end features like Ethernet, SD-card writing, and FAT filesystem
support.

### ST "heavy" HAL compiles, but is not a first-class citizen.
It's included (at least for now) for its USB and SD drivers. At some point those
components may be replaced, at which point there is little reason to include the
rest of the ST HAL. There are some hacks around SysTick handling to allow the
HAL drivers to operate without ever calling HAL_Init().

### Use homemade "lightweight" peripheral drivers whenever possible.
The ST-provided drivers are typically very difficult to read and provide for
quite a bit of mostly-useless flexibility. They also tend to require too much
compile-time configuration in a million nooks and crannies that are difficult
to track down. So instead, let's write and use our own. The Ethernet driver,
for example, ends up a fourth of the source code length and higher performance.