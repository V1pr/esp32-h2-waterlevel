/**
 * Optional zigbee2mqtt external converter.
 *
 * The stock 872WZ definition already decodes liquid_depth, liquid_level_percent
 * and liquid_state. It is marked as mains powered and does not publish battery.
 * This file is the same profile plus the standard Power Configuration battery
 * attributes this firmware reports.
 *
 * Copy it to zigbee2mqtt's external converter folder and restart zigbee2mqtt:
 *   data/external_converters/872WZ-battery.js
 *
 * Water level still works if you skip this file. Battery percentage and
 * voltage show up in MQTT only after it is installed.
 */
const {battery} = require("zigbee-herdsman-converters/lib/modernExtend");
const tuya = require("zigbee-herdsman-converters/lib/tuya");
const exposes = require("zigbee-herdsman-converters/lib/exposes");

const e = exposes.presets;
const ea = exposes.access;

const definition = {
    fingerprint: [{modelID: "TS0601", manufacturerName: "_TZE204_7yyuo8sr"}],
    model: "872WZ",
    vendor: "Tuya",
    description: "Water level sensor",
    extend: [
        battery({voltage: true}),
        tuya.modernExtend.tuyaBase({dp: true, forceTimeUpdates: true, queryOnConfigure: true}),
    ],
    exposes: [
        e.enum("liquid_state", ea.STATE, ["low", "normal", "high"]).withDescription("Liquid level status"),
        e.numeric("liquid_level_percent", ea.STATE).withUnit("%").withDescription("Liquid level ratio"),
        e.numeric("liquid_depth", ea.STATE).withUnit("m").withDescription("Liquid depth"),
        e
            .numeric("max_set", ea.STATE_SET)
            .withUnit("%")
            .withValueMin(0)
            .withValueMax(100)
            .withValueStep(1)
            .withDescription("Liquid maximum percentage"),
        e
            .numeric("min_set", ea.STATE_SET)
            .withUnit("%")
            .withValueMin(0)
            .withValueMax(100)
            .withValueStep(1)
            .withDescription("Liquid minimum percentage"),
        e
            .numeric("liquid_depth_max", ea.STATE_SET)
            .withUnit("m")
            .withValueMin(0)
            .withValueMax(5)
            .withValueStep(0.01)
            .withDescription("Maximum liquid depth"),
        e
            .numeric("installation_height", ea.STATE_SET)
            .withUnit("m")
            .withValueMin(0)
            .withValueMax(5)
            .withValueStep(0.01)
            .withDescription("Distance from the sensor to the bottom of the tank"),
        e.binary("silent_mode", ea.STATE_SET, "ON", "OFF").withDescription("Stored only; this device has no buzzer"),
        e
            .numeric("alarm_duration", ea.STATE_SET)
            .withUnit("min")
            .withValueMin(1)
            .withValueMax(60)
            .withValueStep(1)
            .withDescription("Stored only; this device has no buzzer"),
    ],
    meta: {
        tuyaDatapoints: [
            [
                1,
                "liquid_state",
                tuya.valueConverterBasic.lookup({
                    low: tuya.enum(1),
                    normal: tuya.enum(0),
                    high: tuya.enum(2),
                }),
            ],
            [2, "liquid_depth", tuya.valueConverter.divideBy100],
            [7, "max_set", tuya.valueConverter.raw],
            [8, "min_set", tuya.valueConverter.raw],
            [9, "silent_mode", tuya.valueConverter.onOff],
            [19, "installation_height", tuya.valueConverter.divideBy100],
            [21, "liquid_depth_max", tuya.valueConverter.divideBy100],
            [22, "liquid_level_percent", tuya.valueConverter.raw],
            [27, "alarm_duration", tuya.valueConverter.raw],
        ],
    },
};

module.exports = definition;
