import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_RADIO_ID = "radio_id"

rf_sniffer_ns = cg.esphome_ns.namespace("rf_sniffer")
RfSniffer = rf_sniffer_ns.class_("RfSniffer", cg.Component)

RFM69x = cg.global_ns.namespace("rfm69x").class_("RFM69x", cg.Component)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RfSniffer),
        cv.Required(CONF_RADIO_ID): cv.use_id(RFM69x),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    # Create the sniffer component
    sniffer = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(sniffer, config)
    
    # Get the radio component reference
    radio = await cg.get_variable(config[CONF_RADIO_ID])
    cg.add(sniffer.set_radio(radio))