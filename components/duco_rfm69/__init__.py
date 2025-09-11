import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome import automation, pins

duco_rfm69_ns = cg.esphome_ns.namespace('duco_rfm69')
DucoRFM69 = duco_rfm69_ns.class_('DucoRFM69', cg.Component, spi.SPIDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DucoRFM69),
    cv.Required("cs_pin"): pins.gpio_output_pin_schema,
}).extend(cv.COMPONENT_SCHEMA).extend(spi.spi_device_schema())

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    cg.add(var.set_cs_pin(await cg.gpio_pin_expression(config["cs_pin"])))
