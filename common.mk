ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)
USEFILE=$(PROJECT_ROOT)/devn-bcm4400.use
LIBS=drvrS pmS
OPTIMIZE_TYPE=TIME
include $(MKFILES_ROOT)/qtargets.mk
