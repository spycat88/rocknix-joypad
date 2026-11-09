obj-m := rocknix-joypad.o

# The "rocknix-singleadc-joypad" layout hands its input device to the ROCKNIX
# adc-keys kernel patch through joypad_input_g, so adc-keys can send BTN_MODE
# to the joypad. Only reference that symbol where the single ADC driver used
# to be built (every device except S922X and RK3588, and not when DEVICE is
# unset), so the module still loads on kernels without the patch.
ifneq ($(DEVICE),$(filter $(DEVICE), S922X RK3588))
ccflags-y += -DROCKNIX_JOYPAD_ADC_KEYS_HOOK
endif
