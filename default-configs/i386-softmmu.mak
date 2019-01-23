# Default configuration for i386-softmmu

CONFIG_VMXNET3_PCI=y
CONFIG_IPMI=y
CONFIG_IPMI_LOCAL=y
CONFIG_IPMI_EXTERN=y
CONFIG_ISA_IPMI_KCS=y
CONFIG_ISA_IPMI_BT=y

# Optional devices:
#
CONFIG_HPET=y
CONFIG_APPLESMC=y
CONFIG_PFLASH_CFI01=y
CONFIG_ISA_DEBUG=y
CONFIG_ISA_TESTDEV=y
CONFIG_TEST_DEVICES=y
CONFIG_SGA=y
CONFIG_PVPANIC=y
CONFIG_MEM_DEVICE=y
CONFIG_NVDIMM=y
CONFIG_ACPI_NVDIMM=y
CONFIG_PXB=y
CONFIG_ACPI_VMGENID=y
CONFIG_PCI_DEVICES=y

# Boards:
#
CONFIG_ISAPC=y
CONFIG_I440FX=y
CONFIG_Q35=y
