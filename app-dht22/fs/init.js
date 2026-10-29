load('api_dht.js');
load('api_timer.js');
load('api_mqtt.js');
load('api_config.js');
load('api_sys.js');

let pin = 5;
let dht = DHT.create(pin, DHT.DHT22);
let lastT = null;
let lastH = null;

let httpReply = ffi('void app_http_reply(void *, int, char *)');
let httpMethodIs = ffi('int app_http_method_is(void *, char *)');
let httpReg = ffi(
  'void mgos_register_http_endpoint(char *, void (*)(void *, int, void *, userdata), userdata)'
);
let MG_EV_HTTP_REQUEST = 100;

print('DHT22 on GPIO', pin);

function remember(value, prev) {
  if (value !== value) return prev;
  return value;
}

function switchWifi(staOn, apOn) {
  let ok = Cfg.set({
    wifi: {
      sta: { enable: staOn },
      ap: { enable: apOn, ssid: 'dht22' },
    },
  });
  print('wifi switch sta', staOn, 'ap', apOn, 'saved', ok);
  if (ok) Sys.reboot(500);
}

MQTT.sub('home/sensors/dht22/cmd', function(conn, topic, msg) {
  print('cmd', topic, msg);
  if (msg === 'ap') switchWifi(false, true);
}, null);

function onSensor(c, ev, evd, ud) {
  if (ev !== MG_EV_HTTP_REQUEST) return;
  if (!httpMethodIs(evd, 'GET')) {
    httpReply(c, 405, '{"ok":false}');
    return;
  }
  httpReply(c, 200, JSON.stringify({ t: lastT, h: lastH }));
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
  let temp = dht.getTemp();
  let hum = dht.getHumidity();
  lastT = remember(temp, lastT);
  lastH = remember(hum, lastH);

  print('Temp:', temp, 'C Hum:', hum, '%');

  if (MQTT.isConnected()) {
    let data = { t: lastT, h: lastH };
    let json = JSON.stringify(data);
    MQTT.pub('home/sensors/dht22', json, 1);
    print('Published:', json);
  }
}, null);
