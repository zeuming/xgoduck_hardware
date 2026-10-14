'use strict';
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(__dirname+'/system_health_panel.html','utf8');
const nodes=new Map([...html.matchAll(/id="([^"]+)"/g)].map(m=>[m[1],{textContent:'',dataset:{}}]));
let ready, now=0, paints=0;
const intervals=[];
const context=vm.createContext({window:{addEventListener:(name,fn)=>{assert.equal(name,'DOMContentLoaded');ready=fn;}},
 document:{getElementById:id=>nodes.get(id)},performance:{now:()=>now},
 setInterval:(fn,ms)=>{assert.equal(ms,2000);intervals.push(fn);},
 render:()=>{paints++;},snapshot:{},Date,Number,Math,Infinity});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);ready();
assert.match(nodes.get('x5HealthStatus').textContent,/未知/);
context.snapshot.system_health={state:'no_limit_observed',reason:'本次采样未见 CPU 热限速',age_ms:20,stale:false,
 zones:[{type:'thermal-cpu',celsius:70.15,trips:[{type:'passive',celsius:95}]}],
 policies:[{current_mhz:300,max_mhz:1500,hardware_max_mhz:1500,current_source:'scaling_cur_freq',governor:'schedutil',cpus:'0 1'}],
 cooling:[{type:'cpufreq-cpu0',state:0,max_state:4}],cpu_busy_percent:5,peak_cpu_celsius:72,
 thermal_limited_samples:0,control:{available:true,mode:'walk',cycle_max_ms:23.1,inference_ms:8.5},monitor_started_at:100};
context.render();assert.equal(paints,1);
assert.equal(nodes.get('x5HealthTemperature').textContent,'70.2 °C');
assert.equal(nodes.get('x5HealthFrequency').textContent,'300 MHz');
assert.equal(nodes.get('x5HealthPanel').dataset.level,'ok','idle DVFS is not overheating');
assert.match(nodes.get('x5HealthDetails').textContent,/驱动请求值/);
assert.match(nodes.get('x5HealthDetails').textContent,/23.1 ms/);
context.snapshot.system_health={...context.snapshot.system_health,state:'thermal_limited',reason:'热限速'};
context.render();assert.equal(nodes.get('x5HealthPanel').dataset.level,'warning');
now=6100;intervals[0]();assert.match(nodes.get('x5HealthStatus').textContent,/未知/);
assert.equal(nodes.get('x5HealthTemperature').textContent,'—','stale values cannot look current');
context.snapshot.system_health={state:'unknown',reason:'不完整',stale:false,age_ms:0};
context.render();assert.equal(nodes.get('x5HealthTemperature').textContent,'—');
assert.equal(nodes.get('x5HealthPanel').dataset.level,'unknown');
console.log('Health UI: values, DVFS, limiting, missing/stale data and timing passed; no fetch or control API available.');
