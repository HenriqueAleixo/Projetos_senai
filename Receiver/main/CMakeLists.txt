set(COMPONENT_SRCS "lora_receiver.c"
                   "lora_crc.c"
				   "mqtt.c"
				   "nvs_storage.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
register_component()