ENTRY(imu_regression_main)
MEMORY {
    FLASH (rx) : ORIGIN = 0x00010000, LENGTH = 1M
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 1M
}
SECTIONS {
    .text : { *(.text*) *(.rodata*) } > FLASH
    .ARM.exidx : { *(.ARM.exidx*) } > FLASH
    .data : { *(.data*) } > RAM
    .bss : { *(.bss*) *(COMMON) } > RAM
    end = .;
    _end = .;
}
