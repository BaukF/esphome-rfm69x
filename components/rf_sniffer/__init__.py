import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import rfm69x
from esphome.const import CONF_ID

AUTO_LOAD = ["rfm69x"]
DEPENDENCIES = ["spi"]

rf_sniffer_ns = cg.esphome_ns.namespace("rf_sniffer")
RFSniffer = rf_sniffer_ns.class_("RFSniffer", cg.Component)

CONF_RFM69X_ID = "rfm69x_id"
CONF_LOG_RAW_PACKETS = "log_raw_packets"
CONF_LOG_FREQUENCY_SCAN = "log_frequency_scan"
CONF_PACKET_FILTER_MIN_LENGTH = "packet_filter_min_length"
CONF_PACKET_FILTER_MAX_LENGTH = "packet_filter_max_length"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RFSniffer),
        cv.GenerateID(CONF_RFM69X_ID): cv.use_id(rfm69x.RFM69x),
        cv.Optional(CONF_LOG_RAW_PACKETS, default=True): cv.boolean,
        cv.Optional(CONF_LOG_FREQUENCY_SCAN, default=False): cv.boolean,
        cv.Optional(CONF_PACKET_FILTER_MIN_LENGTH, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_PACKET_FILTER_MAX_LENGTH, default=255): cv.int_range(min=0, max=255),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    rfm69x_var = await cg.get_variable(config[CONF_RFM69X_ID])
    cg.add(var.set_rfm69x(rfm69x_var))
    
    cg.add(var.set_log_raw_packets(config[CONF_LOG_RAW_PACKETS]))
    cg.add(var.set_log_frequency_scan(config[CONF_LOG_FREQUENCY_SCAN]))
    cg.add(var.set_packet_filter_min_length(config[CONF_PACKET_FILTER_MIN_LENGTH]))
    cg.add(var.set_packet_filter_max_length(config[CONF_PACKET_FILTER_MAX_LENGTH]))