load('api_ds18b20.js');
load('api_timer.js');
load('api_mqtt.js');
load('api_config.js');
load('api_sys.js');

let lastT = null;

let httpReply = ffi('void app_http_reply(void *, int, char *)');
let httpMethodIs = ffi('int app_http_method_is(void *, char *)');
let httpReg = ffi(
  'void mgos_register_http_endpoint(char *, void (*)(void *, int, void *, userdata), userdata)'
);
let MG_EV_HTTP_REQUEST = 100;

if (Cfg.get('ds18b20.pin') !== 4 || !Cfg.get('ds18b20.enable')) {
  Cfg.set({ ds18b20: { enable: true, pin: 4 } });
  print('ds18b20 pin set to 4, reboot');
  Sys.reboot(500);
}

print('DS18B20 on GPIO', Cfg.get('ds18b20.pin'));

function switchWifi(staOn, apOn) {
  let ok = Cfg.set({
    wifi: {
      sta: { enable: staOn },
      ap: { enable: apOn, ssid: 'ds18b20' },
    },
  });
  print('wifi switch sta', staOn, 'ap', apOn, 'saved', ok);
  if (ok) Sys.reboot(500);
}

MQTT.sub('home/sensors/ds18b20/cmd', function(conn, topic, msg) {
  print('cmd', topic, msg);
  if (msg === 'ap') switchWifi(false, true);
}, null);

function onSensor(c, ev, evd, ud) {
  if (ev !== MG_EV_HTTP_REQUEST) return;
  if (!httpMethodIs(evd, 'GET')) {
    httpReply(c, 405, '{"ok":false}');
    return;
  }
  httpReply(c, 200, JSON.stringify({ t: lastT }));
}

function onBack(c, ev, evd, ud) {
  if (ev !== MG_EV_HTTP_REQUEST) return;
  if (!httpMethodIs(evd, 'POST')) {
    httpReply(c, 405, '{"ok":false}');
    return;
  }
  httpReply(c, 200, '{"ok":true}');
  switchWifi(true, false);
}

httpReg('/sensor', onSensor, {});
httpReg('/back', onBack, {});

Timer.set(5000, true, function() {
  let connected = DS18B20.connected();
  if (connected) lastT = DS18B20.get();
  print('Temp:', lastT, 'C connected:', connected);

  if (MQTT.isConnected()) {
    let json = JSON.stringify({ t: lastT });
    MQTT.pub('home/sensors/ds18b20', json, 1);
    print('Published:', json);
  }
}, null);
