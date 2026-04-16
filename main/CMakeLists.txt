idf_component_register(
  SRCS
  "main.cpp"
  "ble_server.c"
  "GpsService.cpp"
  "gps_gatt_def.c"
  "active_look.cpp"
  "ble_manager.cpp"
  INCLUDE_DIRS
  "../include"
  REQUIRES
  bt
  nvs_flash
  esp_driver_uart
  log
  freertos)

