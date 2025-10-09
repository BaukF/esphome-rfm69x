import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.components import spi
from esphome.const import (
            CONF_ID,
            CONF_FREQUENCY,
            CONF_PIN
)

DEPENDENCIES = ["spi"]

rfm69x_ns = cg.esphome_ns.namespace("rfm69x")
RFM69x = rfm69x_ns.class_(
    "RFM69x", cg.Component, spi.SPIDevice
)

CONF_RAW_CODES = "raw_codes"
CONF_RESET_PIN = "reset_pin"
CONF_FREQUENCY = "frequency"
CONF_PROMISCUOUS_MODE = "promiscuous_mode"
CONF_PLL_TIMEOUT = "pll_timeout"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RFM69x),
            cv.Optional(CONF_RAW_CODES, default=False): cv.boolean,
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_FREQUENCY): cv.positive_int,
            cv.Optional(CONF_PROMISCUOUS_MODE, default=False): cv.boolean,
            cv.Optional(CONF_PLL_TIMEOUT, default=50): cv.positive_int,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    if config[CONF_RAW_CODES]:
        cg.add(var.set_raw_codes(config[CONF_RAW_CODES]))
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))
    if config[CONF_PROMISCUOUS_MODE]:
        cg.add(var.set_promiscuous_mode(config[CONF_PROMISCUOUS_MODE]))
    if CONF_FREQUENCY in config:
        cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    if CONF_PLL_TIMEOUT in config:
        cg.add(var.set_pll_timeout(config[CONF_PLL_TIMEOUT]))
