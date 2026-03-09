import {Zcl} from 'zigbee-herdsman';
import * as m from 'zigbee-herdsman-converters/lib/modernExtend';
import * as exposes from "zigbee-herdsman-converters/lib/exposes";
import * as utils from "zigbee-herdsman-converters/lib/utils";
import * as reporting from "zigbee-herdsman-converters/lib/reporting";

const e = exposes.presets;
const ea = exposes.access;

const rescanExtend = {
  rescan: () => {
    const exposes = [
      e
        .enum("rescan", ea.SET, ["rescan"])
        .withDescription("Rescan OneWire devices")
    ]
    const toZigbee = [
      {
        key: ["rescan"],
        convertSet: async (entity, key, value, meta) => {
          await entity.command("configure", 0x80, {}, utils.getOptions(meta.mapped, entity))
        }
      }
    ]
    return {
      exposes,
      fromZigbee: [],
      toZigbee,
      isModernExtend: true
    }
  }
};

export function deviceId(endpoint) {
    return m.text({
        name: "deviceId",
        cluster: "msTemperatureMeasurement",
        attribute: "deviceId",
        endpointName: endpoint,
        description: "OneWire device ID",
        access: "STATE_GET",
    });
}

export default {
    zigbeeModel: ['Mahtan_OW_DIY'],
    model: 'Mahtan_OW_DIY',
    vendor: 'Mahtan-DIY',
    description: 'Mahtan OneWire sensor',
    extend: [
        m.deviceAddCustomCluster("configure", {
          ID: 0xFC00,
          commands: {
              rescan: {
                  ID: 0x80,
                  parameters: [],
              },
          },
          commandsResponse: {},
        }),
        m.deviceAddCustomCluster("msTemperatureMeasurement", {
          ID: 0x0402,
          attributes: {
            deviceId: {ID: 0xF001, type: Zcl.DataType.TEXT, manufacturerCode: 0x1234},
          },
        }),
        m.deviceAddCustomCluster("msPressureMeasurement", {
          ID: 0x0403,
        }),
        m.deviceEndpoints({
          "endpoints":{
            "1":1,"2":2,"3":3,
            "10":10,"11":11,"12":12,"13":13,"14":14,"15":15,"16":16,"17":17,"18":18,"19":19,
            "20":20,"21":21,
          }
        }),
        m.onOff({"powerOnBehavior":false,"endpointNames":["2","3"]}),
        m.numeric({
          name: "temperature",
          cluster: "msTemperatureMeasurement",
          attribute: "measuredValue",
          reporting: {min: 10, max: 300, change: 50},
          description: "Measured temperature value",
          unit: "°C",
          scale: 100,
          access: "STATE_GET",
          endpointNames: ["10","11","12","13","14","15","16","17","18","19"],
        }),
        deviceId("10"),
        deviceId("11"),
        deviceId("12"),
        deviceId("13"),
        deviceId("14"),
        deviceId("15"),
        deviceId("16"),
        deviceId("17"),
        deviceId("18"),
        deviceId("19"),
        m.numeric({
          name: "pressure",
          cluster: "msPressureMeasurement",
          attribute: "measuredValue",
          reporting: {min: 10, max: 300, change: 50},
          description: "Measured pressure value",
          unit: "kPa",
          scale: 10,
          access: "STATE_GET",
          endpointNames: ["20","21"],
        }),
        rescanExtend.rescan()],
};
