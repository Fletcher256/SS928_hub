$ErrorActionPreference = 'Stop'

$Root = Resolve-Path (Join-Path $PSScriptRoot '..')
$LocalToolchain = Join-Path $Root 'tools/gcc-arm/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin'
if (Test-Path (Join-Path $LocalToolchain 'arm-none-eabi-gcc.exe')) {
  $CC = Join-Path $LocalToolchain 'arm-none-eabi-gcc.exe'
  $Objcopy = Join-Path $LocalToolchain 'arm-none-eabi-objcopy.exe'
  $Size = Join-Path $LocalToolchain 'arm-none-eabi-size.exe'
} else {
  $CC = 'arm-none-eabi-gcc'
  $Objcopy = 'arm-none-eabi-objcopy'
  $Size = 'arm-none-eabi-size'
}
$BuildDir = Join-Path $Root 'build/gcc'

New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null

$CommonFlags = @(
  '-mcpu=cortex-m3',
  '-mthumb',
  '-O2',
  '-ffunction-sections',
  '-fdata-sections',
  '-fno-common',
  '-Wall',
  '-Wno-unused-variable',
  '-Wno-unused-function',
  '-Wno-unused-but-set-variable',
  '-Wno-sign-compare',
  '-std=gnu99',
  '-DUSE_STDPERIPH_DRIVER',
  '-DSTM32F10X_MD',
  '-I.',
  '-ILIBS',
  '-ISTART',
  '-IUSER',
  '-ICore',
  '-IHARDWARE',
  '-IDMP'
)

$Sources = @(
  'LIBS/misc.c',
  'LIBS/stm32f10x_adc.c',
  'LIBS/stm32f10x_bkp.c',
  'LIBS/stm32f10x_can.c',
  'LIBS/stm32f10x_cec.c',
  'LIBS/stm32f10x_crc.c',
  'LIBS/stm32f10x_dac.c',
  'LIBS/stm32f10x_dbgmcu.c',
  'LIBS/stm32f10x_dma.c',
  'LIBS/stm32f10x_exti.c',
  'LIBS/stm32f10x_flash.c',
  'LIBS/stm32f10x_fsmc.c',
  'LIBS/stm32f10x_gpio.c',
  'LIBS/stm32f10x_i2c.c',
  'LIBS/stm32f10x_iwdg.c',
  'LIBS/stm32f10x_pwr.c',
  'LIBS/stm32f10x_rcc.c',
  'LIBS/stm32f10x_rtc.c',
  'LIBS/stm32f10x_sdio.c',
  'LIBS/stm32f10x_spi.c',
  'LIBS/stm32f10x_tim.c',
  'LIBS/stm32f10x_usart.c',
  'LIBS/stm32f10x_wwdg.c',
  'START/system_stm32f10x.c',
  'USER/stm32f10x_it.c',
  'main.c',
  'Core/Timers.c',
  'HARDWARE/PWMO.c',
  'HARDWARE/USART.c',
  'HARDWARE/Keys.c',
  'HARDWARE/Motors.c',
  'HARDWARE/MPU6050.c',
  'HARDWARE/_MyI2C_.c',
  'HARDWARE/filter.c',
  'HARDWARE/generic.c',
  'HARDWARE/LED.c',
  'build_gcc/syscalls.c'
)

Push-Location $Root
try {
  $Objects = @()
  foreach ($Source in $Sources) {
    $ObjName = ($Source -replace '[\\/]', '_') -replace '\.c$', '.o'
    $Obj = Join-Path $BuildDir $ObjName
    & $CC @CommonFlags -c $Source -o $Obj
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    $Objects += $Obj
  }

  $StartupObj = Join-Path $BuildDir 'startup_stm32f103_md_gcc.o'
  & $CC @CommonFlags -c 'build_gcc/startup_stm32f103_md_gcc.s' -o $StartupObj
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
  $Objects += $StartupObj

  $Elf = Join-Path $BuildDir 'SS928_hub.elf'
  $Hex = Join-Path $BuildDir 'SS928_hub.hex'
  $Bin = Join-Path $BuildDir 'SS928_hub.bin'
  $Map = Join-Path $BuildDir 'SS928_hub.map'

  & $CC @CommonFlags $Objects `
    '-Tbuild_gcc/stm32f103c8.ld' `
    '-Wl,--gc-sections' `
    "-Wl,-Map=$Map" `
    '-Wl,--print-memory-usage' `
    '-Wl,-u,_printf_float' `
    '--specs=nano.specs' `
    '-lm' `
    -o $Elf
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

  & $Objcopy -O ihex $Elf $Hex
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
  & $Objcopy -O binary $Elf $Bin
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
  & $Size $Elf

  Write-Host "Built:"
  Write-Host "  $Elf"
  Write-Host "  $Hex"
  Write-Host "  $Bin"
} finally {
  Pop-Location
}
