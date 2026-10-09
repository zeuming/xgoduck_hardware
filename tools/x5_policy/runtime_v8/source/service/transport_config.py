"""Explicit local SSH transport configuration; import performs no device access.

Changing the network address never changes the reviewed board identity or the
canonical, pinned SSH host key. USB keeps its frozen route parser. Wi-Fi is an
explicit opt-in to one private IPv4 address through the reviewed en0 interface.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import ipaddress
import re

CANONICAL_HOST = '192.168.128.10'
_PRIVATE_NETWORKS = tuple(ipaddress.IPv4Network(value) for value in
                          ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))
_MAX_ROUTE_TEXT = 16384


@dataclass(frozen=True)
class TransportConfig:
    kind: str = 'usb'
    host: str = CANONICAL_HOST
    interface: str = 'en5'
    user: str = 'root'
    port: int = 22
    host_key_alias: str = CANONICAL_HOST

    def __post_init__(self):
        if (type(self.kind) is not str or self.kind not in ('usb', 'wifi')
                or type(self.host) is not str or type(self.interface) is not str
                or self.user != 'root' or type(self.user) is not str
                or type(self.port) is not int or self.port != 22
                or type(self.host_key_alias) is not str or self.host_key_alias != CANONICAL_HOST):
            raise ValueError('Invalid reviewed SSH transport configuration')
        if self.kind == 'usb':
            if self.host != CANONICAL_HOST or self.interface != 'en5':
                raise ValueError('Reviewed USB address and en5 interface cannot change')
        else:
            try:
                address = ipaddress.IPv4Address(self.host)
            except ValueError as error:
                raise ValueError('Wi-Fi requires an explicit private IPv4 address') from error
            if (str(address) != self.host or not any(address in network for network in _PRIVATE_NETWORKS)
                    or self.host == CANONICAL_HOST or self.interface != 'en0'):
                raise ValueError('Wi-Fi requires an explicit private IPv4 address through en0')

    @property
    def target(self):
        return self.user + '@' + self.host

    def route_command(self):
        return ['route', '-n', 'get', self.host]

    def validate_route(self, text, helper=None):
        if type(text) is not str or not text or len(text) > _MAX_ROUTE_TEXT or '\0' in text:
            raise ValueError('Reviewed ' + ('USB' if self.kind == 'usb' else 'Wi-Fi') + ' route is unavailable')
        if self.kind == 'usb':
            if not isinstance(helper, dict) or not callable(helper.get('parse_route')):
                raise ValueError('Reviewed USB route parser is unavailable')
            if helper['parse_route'](text) != 'en5':
                raise ValueError('Reviewed USB route is unavailable')
            return 'en5'
        interfaces = re.findall(r'^\s*interface:\s*(\S+)\s*$', text, re.M)
        if interfaces != ['en0']:
            raise ValueError('Reviewed Wi-Fi route is unavailable; expected one en0 interface')
        return 'en0'

    def verify_route(self, result, helper=None):
        if type(getattr(result, 'returncode', None)) is not int or result.returncode != 0:
            raise ValueError('Reviewed ' + ('USB' if self.kind == 'usb' else 'Wi-Fi') + ' route is unavailable')
        return self.validate_route(getattr(result, 'stdout', None), helper)

    def ssh_options(self):
        # The known_hosts file retains its exact canonical single-host row.
        # StrictHostKeyChecking and its fingerprint checks remain at each caller.
        return ['-p', str(self.port), '-o', 'HostKeyAlias=' + self.host_key_alias]

    def as_record(self):
        return asdict(self)


DEFAULT_TRANSPORT = TransportConfig()


def resolve_transport(value=None):
    if value is None:
        return DEFAULT_TRANSPORT
    if type(value) is not TransportConfig:
        raise ValueError('Explicit reviewed TransportConfig required')
    return value
