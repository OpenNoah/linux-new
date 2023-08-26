obj-$(CONFIG_MAC802154) +=	mac802154.o
mac802154-objs		:= rx.o tx.o main.o monitor.o wpan.o mac_cmd.o scan.o mib.o \
			beacon.o beacon_hash.o smac.o

ccflags-$(CONFIG_MAC802154_DEBUG) += -DDEBUG
ccflags-y += -Wall
