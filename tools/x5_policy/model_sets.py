"""Explicit selection of original or locally trained four-policy sets.

Local adaptation to the upstream single model directory: only asset selection
changes here. Observation layout, HOME, gains, EMA, limits and motion lifecycle
remain in the existing controller. CPU loading never sends a motor command.
"""
import copy
import hashlib
import importlib
from pathlib import Path
import secrets

LABELS = {'original': '原版模型', 'trained': '自训练模型'}
MODEL_NAMES = ('walk', 'getup', 'pick', 'roulade')
DIRECTORIES = {'original': 'policy/models', 'trained': 'policy/model-sets/trained'}


class ModelSetMixin:
    def model_set_status(self):
        current = getattr(self, '_model_set', 'original')
        catalog = self.deployment.get('model_sets', {'original': {}})
        return dict(model_set=current, model_set_label=LABELS[current],
                    available_model_sets=[name for name in LABELS if name in catalog])

    def _select_model_set(self, requested):
        """Called under the existing connection lifecycle lock, before UART.

        An existing connection must be explicitly closed before replacing its
        sessions. Never unload a running controller or silently stop a robot.
        All four sessions are staged and verified before the selection changes.
        """
        if requested is None:
            return
        if type(requested) is not str or requested not in LABELS:
            raise ValueError('未知模型组')
        if requested == getattr(self, '_model_set', 'original'):
            return
        with self.lock:
            if self._closed or self.phase == 'deploying':
                raise ValueError('当前服务正在切换连接，请等待结果')
            if self._owns_locked():
                raise ValueError('切换模型前请先点击「停止并断开运控」；当前模型未更换')
            old_controller = self._controller
            if old_controller is not None and (not self.closed_proof
                    or not self.closed_proof.get('serial_closed')
                    or self._namespace is None
                    or not self._serial_closed(old_controller, self._namespace)):
                raise ValueError('旧串口尚未关闭，当前模型未更换')
            declaration = copy.deepcopy(self.deployment.get('model_sets', {}).get(requested))
        if not declaration or set(declaration.get('model_hashes', {})) != set(MODEL_NAMES):
            raise ValueError('该模型组尚未安装完整的四个模型')
        directory = self.directory / DIRECTORIES[requested]
        if directory.is_symlink() or directory.resolve(strict=True) != directory:
            raise ValueError('模型目录身份不匹配')
        infer = importlib.import_module('infer')
        sessions = {}
        for name in MODEL_NAMES:
            path = directory / ('xgoduck_' + name + '.onnx')
            if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != declaration['model_hashes'][name]:
                raise ValueError('模型校验失败：' + name)
            session = infer.session(name, model_directory=directory)
            if session.get_providers() != ['CPUExecutionProvider']:
                raise ValueError('模型必须使用CPU推理')
            observation = infer.np.zeros((1, 61), infer.np.float32)
            observation[0, 5] = -1
            infer.infer(session, observation)
            sessions[name] = session
        # Preserve the installed execution mode. Do not introduce an async
        # pipeline into a synchronous runtime. If a worker is already used,
        # replace it using the SAME selected directory and pinned hashes.
        old_worker = getattr(self, '_policy_worker', None)
        worker = None
        if old_worker is not None:
            from policy_worker import PolicyWorker
            worker = PolicyWorker(directory, declaration['model_hashes'])
        try:
            with self.lock:
                if self._owns_locked() or self._controller is not old_controller:
                    raise ValueError('模型加载期间连接已变化，当前模型未更换')
                self.sessions = sessions
                self.deployment['model_hashes'] = copy.deepcopy(declaration['model_hashes'])
                self._model_set = requested
                self._model_directory = directory
                self._policy_worker = worker
        except BaseException:
            if worker is not None:
                worker.close()
            raise
        if old_worker is not None:
            old_worker.close()
        self._record('models-' + secrets.token_hex(8) + '.json',
            dict(self.model_set_status(), model_hashes=declaration['model_hashes'],
                 serial_open_attempts=0, motor_start_attempted=False))
