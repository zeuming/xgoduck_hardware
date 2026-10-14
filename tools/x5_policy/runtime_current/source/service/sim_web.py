"""Bounded same-origin gateway to the X5-local simulation service."""
import http.client
import re
import secrets
import threading

_slots = threading.BoundedSemaphore(6)
GET_API = {'/api/model', '/api/replays', '/api/live'}
POST_API = {'/api/simulate', '/api/live/connect', '/api/live/disconnect'}


def resource_path(request_path, *, post=False):
    """Allow bounded display-only parameters on the viewer document alone.

    They remain in the browser URL; the backend receives only the document
    path. API routes, asset paths and all POST requests keep the original
    no-query contract. Never interpret a query as a hardware operation.
    """
    if '?' not in request_path:
        return request_path
    path, query = request_path.split('?', 1)
    if post or path != '/sim/' or len(query) > 32:
        raise ValueError('仅仿真页面入口支持关节定位参数')
    seen = set()
    for field in query.split('&'):
        match = re.fullmatch(r'(joint|v)=([0-9]{1,3})', field)
        if match is None or match[1] in seen:
            raise ValueError('仿真页面定位参数无效')
        seen.add(match[1])
    return path


def toolbar(page):
    marker = '<nav aria-label="选择工作台功能">'.encode()
    if page.count(marker) != 1:
        raise ValueError('Workbench navigation changed')
    link = '<a class="nav-item" href="/sim/" target="_blank" rel="noopener"><span class="nav-icon">◇</span><span>仿真工作台 ↗</span></a>'
    return page.replace(marker, marker + link.encode(), 1)


def route(handler, frontend, web, *, post=False):
    if not (handler.path == '/sim' or handler.path.startswith('/sim/')):
        return False
    acquired = False
    connection = None
    try:
        try:
            normalized = resource_path(handler.path, post=post)
            original = handler.path
            try:
                handler.path = normalized
                size = handler.framing(post=post)
            finally:
                handler.path = original
        except ValueError as error:
            handler.reply(400, dict(error=str(error)))
            return True
        path = normalized[4:] or '/'
        if path == '/' and not normalized.endswith('/'):
            handler.send_response(302); handler.send_header('Location', '/sim/')
            handler.send_header('Content-Length', '0'); handler.end_headers()
            return True
        cookie = None
        if not post and path == '/' and not frontend.authentication_required:
            token, created = frontend.anonymous_session(handler.headers.get('Cookie'))
            if created: cookie = web.session_cookie(token)
        elif not frontend.authenticated(handler.headers.get('Cookie')):
            handler.reply(401, dict(error='请先打开工作台首页建立会话，再打开仿真工作台。'))
            return True
        if post:
            if path not in POST_API:
                handler.reply(404, dict(error='未知仿真操作')); return True
            if not secrets.compare_digest(handler.headers.get('X-CSRF-Token', '').encode(), frontend.token.encode()):
                handler.reply(403, dict(error='页面会话已变化，请刷新')); return True
        elif (path not in GET_API and path != '/' and
              not (path.endswith(('.js', '.css', '.stl', '.txt')) and '..' not in path and '%' not in path)):
            handler.reply(404, dict(error='未知仿真资源')); return True
        acquired = _slots.acquire(blocking=False)
        if not acquired:
            handler.reply(503, dict(error='仿真请求繁忙，请稍后重试')); return True
        body = handler.rfile.read(size) if post else None
        if post and len(body) != size: raise ValueError('请求正文不完整')
        connection = http.client.HTTPConnection('127.0.0.1', 8788, timeout=45 if path == '/api/simulate' else 5)
        connection.request('POST' if post else 'GET', path, body=body,
                           headers={'Content-Type': 'application/json'} if post else {})
        response = connection.getresponse()
        payload = response.read(16 * 1024 * 1024 + 1)
        if len(payload) > 16 * 1024 * 1024: raise ValueError('仿真响应过大')
        handler.reply(response.status, payload, kind=response.getheader('Content-Type', 'application/octet-stream'), cookie=cookie)
    except (OSError, ValueError, http.client.HTTPException):
        handler.reply(503, dict(error='仿真服务暂不可用，请稍后重试。'))
    finally:
        if connection: connection.close()
        if acquired: _slots.release()
    return True
