"""Remove browser motion lifecycle and local admission from rendered workbench."""
from pathlib import Path
import re


def model_actions_page(text):
    """Give every loaded official policy an equally visible explicit entry."""
    models = (
        ('walk', '行走 / 零速平衡', 'motionWalk', 'X · 3', '速度回零后继续平衡，可用方向按钮或手柄前进。'),
        ('getup', '起身', 'motionGetup', 'L2 · 8', '单次运行，含默认姿态阶段；最多 8 秒，结束后停止卸力。'),
        ('pick', '俯身 / 抓取', 'motionPick', 'R2 · 9', '单次运行 4 秒，包含嘴部开合，结束后停止卸力。'),
        ('roulade', '翻滚', 'motionRoulade', '右摇杆按下 · 14', '单次运行约 1.9 秒，结束后停止卸力。'),
    )
    cards = []
    for name, label, button_id, key, description in models:
        pattern = r'<button\b[^>]*id="' + button_id + r'"[^>]*>.*?</button>'
        matches = list(re.finditer(pattern, text))
        assert len(matches) == 1, 'Model action source changed: ' + name
        button = matches[0].group()
        text = text[:matches[0].start()] + text[matches[0].end():]
        cards.append(f'''<article class="model-action-card" id="modelCard-{name}">
<h4>{label} <small>{name}</small></h4>
<p class="note">{description}</p><p class="source-note">手柄：{key}（以实时编号为准）</p>
<p class="note" id="modelState-{name}" role="status">等待 X5 模型状态</p>
<div class="settings-actions">{button}<button class="button" id="modelPreview-{name}" type="button">预览 · 不动作</button></div>
</article>''')
    anchor = '<div class="capability-note" role="status" aria-live="polite"><strong id="motionStatusTitle">'
    assert text.count(anchor) == 1, 'Model status source changed'
    section = '''<style>
.model-action-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin:16px 0}
.model-action-card{padding:16px;border:1px solid var(--line,#ddd);border-radius:12px;background:var(--paper,#fff)}
.model-action-card h4{margin:0 0 8px}.model-action-card small{font-weight:400;color:var(--muted,#667)}
.model-action-card[data-running="true"]{border-color:#08786c;background:#eef8f4}
@media(max-width:640px){.model-action-grid{grid-template-columns:1fr}}
</style><h3>全部官方模型</h3>
<p class="note">各模型可单独预览和启动。起身、翻滚沿用下方防跌确认；模型加载不代表实机动作已验证。</p>
<div class="model-action-grid" id="motionAllModels">''' + ''.join(cards) + '</div>\n'
    # Keep the established confirmation next to the actions it qualifies.
    confirmation = re.search(r'<div class="checklist"><label><input id="motionBehaviorConfirmed".*?</div>', text)
    assert confirmation, 'Behavior confirmation source changed'
    section += confirmation.group() + '\n'
    text = text[:confirmation.start()] + text[confirmation.end():]
    return text.replace(anchor, section + anchor, 1)


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
