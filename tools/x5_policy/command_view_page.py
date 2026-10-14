"""Remove browser motion lifecycle and local admission from rendered workbench."""
from pathlib import Path
import re


def model_actions_page(text):
    """Collect explicit motion controls and their existing gamepad bindings."""
    models = (
        ('walk', '行走 / 零速平衡', 'motionWalk', 'X · 3'),
        ('getup', '起身 · 单次', 'motionGetup', 'L2 · 8'),
        ('pick', '俯身 / 抓取 · 单次', 'motionPick', 'R2 · 9'),
        ('roulade', '翻滚 · 单次', 'motionRoulade', '右摇杆按下 · 14'),
    )

    def take_button(button_id):
        nonlocal text
        pattern = r'<button\b[^>]*id="' + button_id + r'"[^>]*>.*?</button>'
        matches = list(re.finditer(pattern, text))
        assert len(matches) == 1, 'Motion button source changed: ' + button_id
        button = matches[0].group()
        text = text[:matches[0].start()] + text[matches[0].end():]
        label = {'motionStand': '进入模型默认站姿', 'motionWalk': '启动行走模型 · 零速',
                 'motionPick': '俯身 / 抓取 · 单次',
                 'walkModelForwardLocked': '持续前进 · 0.30 m/s'}.get(button_id)
        if label:
            button = re.sub(r'(<button\b[^>]*>).*?(</button>)',
                            lambda match: match[1] + label + match[2], button)
        return button

    def control(button_id, key, model=None):
        identity = f' id="modelCard-{model}"' if model else ''
        state = (f'<small id="modelState-{model}" class="motion-control-state" role="status">等待模型状态</small>'
                 if model else '')
        shortcut = '仅网页操作' if key == '仅网页' else f'手柄 <kbd>{key}</kbd>'
        return (f'<div class="motion-control-item"{identity}>{take_button(button_id)}'
                f'<span class="motion-shortcut">{shortcut}</span>{state}</div>')

    connection = control('motionConnect', 'L1 · 6') + control('motionRecheck', '仅网页')
    actions = control('motionStand', 'A · 0 / START · 11')
    actions += ''.join(control(button_id, key, name) for name, _, button_id, key in models)
    movement = control('walkModelForwardLocked', 'Y · 4') + control('motionZeroCommand', 'B · 1')
    stops = control('motionStop', 'SELECT · 10') + control('motionDisconnect', 'R1 · 7')
    previews = ''.join(f'<button class="button" id="modelPreview-{name}" type="button">{label.replace(" · 单次", "")}预览</button>'
                       for name, label, _, _ in models)
    # Preserve the original nodes and handlers for the generic preview entry.
    preview_label = re.search(r'<label\b[^>]*>不动作预览模型 <select id="motionPreviewModel".*?</label>', text)
    assert preview_label, 'Preview selector source changed'
    preview_selector = preview_label.group()
    text = text[:preview_label.start()] + text[preview_label.end():]
    legacy_preview = take_button('motionShadow') + take_button('walkModelPreview')
    confirmation = re.search(r'<div class="checklist"><label><input id="motionBehaviorConfirmed".*?</div>', text)
    assert confirmation, 'Behavior confirmation source changed'
    confirmation_html = confirmation.group()
    text = text[:confirmation.start()] + text[confirmation.end():]
    drive = re.search(r'<div id="motionDrive" class="motion-drive" hidden>.*?</div>\s*</div>', text, re.S)
    assert drive, 'Direction controls source changed'
    direction_controls = drive.group().replace(
        '按住移动，松开恢复零速度平衡。切换功能、离开页面或连接超时会停止运动。',
        '行走模型运行时可用。点击后持续执行，点击回零速结束移动。方向操作仅支持网页，手柄摇杆与十字键未映射。')
    for before, after in (('按住前进', '前进 · 0.10 m/s'), ('按住后退', '后退 · 0.10 m/s'),
                          ('按住左转', '左转 · 0.30 rad/s'), ('按住右转', '右转 · 0.30 rad/s')):
        direction_controls = direction_controls.replace(before, after)
    text = text[:drive.start()] + text[drive.end():]
    result_log = re.search(r'<pre\b[^>]*id="motionCommandResults"[^>]*>.*?</pre>', text)
    assert result_log, 'Command results source changed'
    log_html = result_log.group()
    text = text[:result_log.start()] + text[result_log.end():]

    section = '''<style>
#motionAllModels{margin:0 0 20px;padding:18px;border:1px solid var(--line,#ddd);border-radius:12px;background:var(--paper,#fff)}
#motionAllModels h3{margin:0 0 6px;font-size:17px}
.motion-control-row{display:flex;gap:14px;align-items:flex-start;padding:12px 0;border-bottom:1px solid var(--line,#ddd)}
.motion-control-row:last-of-type{border-bottom:0}.motion-control-label{flex:0 0 48px;padding-top:10px;font-size:13px;color:var(--muted,#667)}
.motion-control-buttons{display:flex;flex:1;flex-wrap:wrap;gap:12px 14px}
.motion-control-item{display:flex;flex-direction:column;align-items:flex-start;gap:5px;max-width:230px}
.motion-control-item .button{margin:0;min-height:42px}.motion-shortcut{font-size:12px;color:var(--muted,#667)}
.motion-shortcut kbd{font:inherit;font-weight:600;color:var(--ink,#263238)}
.motion-control-state{font-size:11px;line-height:1.5;color:var(--muted,#667);max-width:210px}
.motion-control-item[data-running="true"] .button{box-shadow:0 0 0 2px #08786c}
#motionAllModels details{margin-top:12px}#motionAllModels .checklist{margin-top:12px}
#motionAllModels .motion-stop{background:#b43123;color:#fff;border-color:#b43123}
@media(max-width:640px){#motionAllModels{padding:14px}.motion-control-row{flex-direction:column;gap:8px}.motion-control-label{flex-basis:auto;padding:0}.motion-control-buttons{gap:12px}.motion-control-item{max-width:100%}}
</style><section id="motionAllModels" aria-labelledby="motionControlsTitle">
<h3 id="motionControlsTitle">运控操作</h3>
<p class="note">按钮下方为对应手柄按键，数字为实时按钮编号；物理键名请以接收器反馈核对。</p>
'''
    for label, buttons in (('连接', connection), ('动作', actions), ('移动', movement), ('停止', stops)):
        section += f'<div class="motion-control-row"><strong class="motion-control-label">{label}</strong><div class="motion-control-buttons">{buttons}</div></div>\n'
    section += confirmation_html + '''
<details><summary>不动作预览 · 仅网页</summary><p class="note">使用真实反馈推理，不发送舵机目标。</p><div class="settings-actions">''' + previews + '''</div>
<div hidden>''' + preview_selector + legacy_preview + '''</div></details>
<details><summary>更多方向控制 · 仅网页</summary>''' + direction_controls + '''</details>
<details><summary>最近操作结果</summary>''' + log_html + '''</details>
<p class="source-note">Y 再按一次可回零速，B 直接回零速；SELECT 停止并释放扭矩。起身、俯身和翻滚为单次动作，结束后卸力。</p>
</section>\n'''
    # The legacy initializer otherwise moves WALK back out of this panel.
    relocation = "el('walkModelTestActions').insertBefore(el('motionWalk'),el('walkModelForwardLocked'));"
    assert text.count(relocation) == 1, 'Walk button relocation source changed'
    text = text.replace(relocation, '', 1)
    text = re.sub(r'<div class="settings-actions">\s*</div>', '', text)
    # Keep the existing detailed walk diagnostics, but fold the long prose.
    start_tag = '<div class="capability-note" id="walkModelTest" aria-labelledby="walkModelTestTitle">'
    assert text.count(start_tag) == 1, 'Walk details source changed'
    start = text.index(start_tag)
    depth = 0
    for tag in re.finditer(r'</?div\b[^>]*>', text[start:]):
        depth += -1 if tag.group().startswith('</') else 1
        if depth == 0:
            end = start + tag.end()
            body = text[start + len(start_tag):end - len('</div>')]
            text = (text[:start] + start_tag + '<details><summary>行走模型说明与运行记录</summary>'
                    + body + '</details></div>' + text[end:])
            break
    else:
        raise AssertionError('Walk details closing tag changed')
    body = text.index('<div class="card-body">', text.index('id="motion-models"')) + len('<div class="card-body">')
    return text[:body] + '\n' + section + text[body:]


def command_view_page(page):
    text = page.decode()
    start = text.index('async function runMotion(action,extra={}){')
    end = text.index('function finiteVector(value)', start)
    text = text[:start] + 'async function runMotion(...args){return commandViewRunMotion(...args);}\n' + text[end:]
    # Remove only motion side effects. Visibility may still pause display GETs.
    prefixes = (
        "globalThis.addEventListener?.('blur',()=>{if(motionFresh()",
        "document.addEventListener?.('visibilitychange',()=>{if(motionIndependentHold()",
        "globalThis.addEventListener?.('pagehide',()=>{if(!localFilePage&&!motionIndependentHold()",
        "  if(workspaceView==='motion-models'&&next!==workspaceView&&!motionIndependentHold()",
    )
    lines = text.splitlines(True)
    for prefix in prefixes:
        matches = [line for line in lines if line.startswith(prefix)]
        assert len(matches) == 1, 'Motion lifecycle source changed: ' + prefix
        lines.remove(matches[0])
    text = ''.join(lines).replace(';setInterval(motionHeartbeatTick,100)', '')
    text = text.replace('  syncMotionConfirmations();\n', '')
    # Remove stale instructions left by the earlier browser-heartbeat workflow.
    text = text.replace('固定站姿由 X5 独立保持，切换、刷新或关闭网页不会打断；点击停止按钮或硬件反馈异常时释放扭矩。机身仍需支撑。平衡与行走等动作离开页面或网页控制超时时停止。',
        '模型动作由 X5 运行；切换、刷新或关闭网页不会发送停止命令。需要结束时，明确点击停止按钮。')
    text = text.replace('固定站姿期间，机身总倾角达到 15° 会停止。启动站立平衡前仍须小于 15°；关节到位不能证明独立站稳，机身仍需支撑。',
        '执行状态、停止原因和舵机扭矩反馈由 X5 发布；请查看下方返回记录。')
    text = text.replace('控制心跳和停止按钮保持独立，读数年龄继续增加。', 'X5 继续运行，读数年龄继续增加。')
    text = text.replace('控制心跳和停止按钮独立。', '运控由 X5 独立运行。')
    # Legacy disabled assignments become display-only for motion buttons.
    # Other workbench forms retain their existing, separately scoped behavior.
    text = text.replace("const state={blocked:button.disabled===true};", """const commandViewMotionButton=()=>/^(motion|walkModel|x5Gamepad)/.test(button.id||'');
  const state={blocked:button.disabled===true};""", 1)
    text = text.replace('get(){return state.blocked;},',
        'get(){return commandViewMotionButton()?false:state.blocked;},', 1)
    text = text.replace('state.blocked=!!value;button.dataset.nativeButtonBlocked',
        'state.blocked=commandViewMotionButton()?false:!!value;button.dataset.nativeButtonBlocked', 1)
    # Motion clicks never enter the old capture-phase display admission gate.
    text = text.replace("  const button=nativeButtonFromEvent(event);if(!button)return;", """  const button=nativeButtonFromEvent(event);if(!button)return;
  if(/^(motion|walkModel|x5Gamepad)/.test(button.id||'')){nativeActivateButton(button);return;}""", 1)
    anchor = re.search(r'<button\b[^>]*id="walkModelForwardLocked"[^>]*>.*?</button>', text)
    assert anchor
    text = text[:anchor.end()] + '''
<button class="button" id="motionZeroCommand" type="button">回零速 · 保持平衡</button>
<pre class="note" id="motionCommandResults" role="status" aria-live="polite" style="white-space:pre-wrap;overflow-wrap:anywhere"></pre>
''' + text[anchor.end():]
    script = Path(__file__).with_name('command_view_ui.js').read_text()
    assert text.count('</script>') == 1
    text = model_actions_page(text)
    return text.replace('</script>', script + '\n</script>', 1).encode()
