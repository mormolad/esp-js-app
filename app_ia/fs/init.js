load('api_timer.js');

print('hello world from js');

Timer.set(1000, true, function() {
  print('hello world from js');
}, null);
