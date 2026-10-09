
import base64, collections, errno, fcntl, hashlib, json, math, os, re, select, signal
import stat, struct, subprocess, sys, termios, time, functools
from pathlib import Path
HEADER = struct.Struct('<2sBBHI')
STATE_PACKET = struct.Struct('<IIHHHBB15H15h6fIBB')
IDS = (10,11,12,13,14,20,21,22,23,24,30,31,32,33,34)
ALL_MASK = (1 << len(IDS)) - 1
REPLY_SECONDS = .20
MAX_KERNEL_BYTES = 2*1024*1024
PERIOD_SECONDS = .02
HEALTH_INTERVAL = .25
CONTROL_END = b'END_FEEDBACK\n'
REVIEWED_HISTORICAL_FAULTS = [{'line': '[    7.573470] systemd-journald[273]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.944418] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 3008.252350] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}]
REVIEWED_BOOT_ID = 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab'
REVIEWED_CID = '035344534b33324785d55cb687017c00'
RECONNECT_PROOF = {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '92f372030bf2b6c034f0b4a85c88c7efafb9f108c5158a534de03767797580da', 'kernel_prefix_sha256': 'e75b1b38daef8e19dfcbf81af282b40e309e8fc02cd1c23342ef759f3ace04de', 'kernel_line_count': 962}
RECONNECT_HISTORICAL_FAULTS = [{'line': '[    7.573470] systemd-journald[273]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.944418] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 3008.252350] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8518.691466] usb 1-1: USB disconnect, device number 2', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8518.691521] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8518.931957] usb 2-1: USB disconnect, device number 2', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8544.480306] txrx_err:<aicwf_sdio_wakeup,1312>: sdio wakeup fail', 'reason': 'Kernel reported an SD/MMC failure'}]
RECONNECT_ROOT = '21 1 179:2 / / rw,relatime shared:1 - ext4 /dev/root rw'
RECONNECT_MODEL = 'D-Robotics RDK X5 V1.0'
RECONNECT_SERIAL = {'path': '/dev/ttyUSB0', 'vid': '1a86', 'pid': '7523', 'sysfs_name': '1-1.3', 'serial_number': ''}
RECONNECT_KERNEL_LINES = 962
RECONNECT_KERNEL_SHA256 = 'e75b1b38daef8e19dfcbf81af282b40e309e8fc02cd1c23342ef759f3ace04de'
RECONNECT_VARIANTS = ({'boot_id': '7d23364b-a37c-4ca9-a52f-56bb90547d88', 'proof': {'kernel_line_count': 924, 'kernel_prefix_sha256': '49dc785846d729879b1eb7bae1b52126509dff56df5b708f28a71b183098f2a8', 'profile_sha256': 'e1e7d9acdbe6b72081378718bc080643bc855260a49e8e0bda71e1a1da4df07a', 'scope': 'feedback_only_reviewed_reconnect'}, 'historical_faults': [{'line': '[    7.491990] systemd-journald[288]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.724435] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}], 'requires_discovery': True}, {'boot_id': 'c24863fc-a8eb-455d-a4f6-54633b6483e3', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '1e4b12d7d8db2abe123ac5717d338a690b93da49ff32d6d2614ca481e8a12b94', 'kernel_prefix_sha256': '486185a5e0943f77ea3bb9ff7407e025c8ced869fd4cd9270dcd4427861f250f', 'kernel_line_count': 968}, 'historical_faults': [{'line': '[    7.385520] systemd-journald[275]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 3469.515263] usb 1-1.4: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 6976.872478] usb 1-1.4: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7006.565426] usb 1-1.4: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7019.620394] usb 1-1.4: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7086.941588] usb 1-1.4: USB disconnect, device number 8', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7089.501031] usb 1-1.4: USB disconnect, device number 9', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7149.911226] usb 1-1.4: USB disconnect, device number 10', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 9685.073183] usb 1-1.4: USB disconnect, device number 11', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'requires_discovery': True}, {'boot_id': 'c24863fc-a8eb-455d-a4f6-54633b6483e3', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'aa40ce44e09ba1bd677dc5c133d3442d4065a0050a653a03136aeea9bb219f3f', 'kernel_prefix_sha256': 'e8ff7b0b3a9456eb1a7c37fe1b73a2e8f10c49f874ae4c589788e07ecc924d48', 'kernel_line_count': 968}, 'historical_faults': [{'line': '[    7.385520] systemd-journald[275]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 3469.515263] usb 1-1.4: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 6976.872478] usb 1-1.4: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7006.565426] usb 1-1.4: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7019.620394] usb 1-1.4: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7086.941588] usb 1-1.4: USB disconnect, device number 8', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7089.501031] usb 1-1.4: USB disconnect, device number 9', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 7149.911226] usb 1-1.4: USB disconnect, device number 10', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'requires_discovery': True}, {'boot_id': 'c24863fc-a8eb-455d-a4f6-54633b6483e3', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'a0e2322015e34108b9b879d749f16ef07dc69b3e42e43468d315e5e8e23298f4', 'kernel_prefix_sha256': '5b4a13362166637c829f9dc7bb9dd11225b0e0ce82f3a10ed387dabc149d82aa', 'kernel_line_count': 944}, 'historical_faults': [{'line': '[    7.385520] systemd-journald[275]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 3469.515263] usb 1-1.4: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'requires_discovery': True}, {'boot_id': 'c24863fc-a8eb-455d-a4f6-54633b6483e3', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '6cbc2bc51cb2a7321b4baa890a8198c1423a0c4ebb21cf7319e4c5e0c1bdc1d8', 'kernel_prefix_sha256': '446afca6b270b70792f41981099aba26345335c056530f404dc6990a3e2166c7', 'kernel_line_count': 934}, 'historical_faults': [{'line': '[    7.385520] systemd-journald[275]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 2860.552862] usb 1-1.4: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'requires_discovery': True}, {'boot_id': 'c24863fc-a8eb-455d-a4f6-54633b6483e3', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'cbc644070503c2c5099cdd94fc692e4d2cabbfb870b1f6f57df24363e84e8143', 'kernel_prefix_sha256': '31cadf6fbc9e50742b35f18f4731be25f25c7b76a22f7613878c6d9e408f66a8', 'kernel_line_count': 924}, 'historical_faults': [{'line': '[    7.385520] systemd-journald[275]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.747187] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}], 'requires_discovery': True}, {'boot_id': '4a15fef9-dcc3-4e59-84a7-e8b62a01e208', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'a69fc3065fe1ffc45ff6457d465e9c92969743c195f255f19f37bd16324035ca', 'kernel_prefix_sha256': '01d3a85c796ce8bb7ba0987ca06e0cad3db4304fefbd76e3731ad2f6fd4f0e9f', 'kernel_line_count': 944}, 'historical_faults': [{'line': '[    7.746460] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.748613] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[  348.417038] usb 1-1.4: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[  386.045233] usb 1-1.4: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'requires_discovery': True}, {'boot_id': '4a15fef9-dcc3-4e59-84a7-e8b62a01e208', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '03fd2bde822b13a57223185d5832c64c83d69a9b69fd50930e8b09b1f66467f3', 'kernel_prefix_sha256': 'a0cc0912ebd982ad206611c1015b30749a15b069eafe515d46a10f9df93a1959', 'kernel_line_count': 924}, 'historical_faults': [{'line': '[    7.746460] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.748613] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}], 'requires_discovery': True}, {'boot_id': 'f2070288-4492-4ddc-9594-ad00640d01b7', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'c3886d7ff81ca51c8760cd2942645246376b9a2b4ddab36abf9c2140ac6cc8b0', 'kernel_prefix_sha256': '36c8abdbbd95399cafa57ecc5ef46d26326f96979bda4f4ddd3aad5901db0ada', 'kernel_line_count': 924}, 'historical_faults': [{'line': '[    7.647480] systemd-journald[276]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.707257] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}], 'requires_discovery': True}, {'boot_id': 'acbc7d13-7515-4419-a55d-90cf267d74b0', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'ce9fa4b1055baaca051a83e9ab58f814600d6cd9ca29efad5f22a76937a032b0', 'kernel_prefix_sha256': '4c55e0485f393e7487ce34c3b8867f10165516d89ff90e436649ee2651d14423', 'kernel_line_count': 922}, 'historical_faults': [{'line': '[    7.379408] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.469697] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}], 'requires_discovery': True}, {'boot_id': 'acbc7d13-7515-4419-a55d-90cf267d74b0', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'b4d9cd347aa87246fce390804173a307ecec9749925ae5b7557ffdbe487dbba9', 'kernel_prefix_sha256': '05af493f614d06e30caa269c477bc455aeddee33f783bd8f769d2e714a3d598a', 'kernel_line_count': 932}, 'historical_faults': [{'line': '[    7.379408] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.469697] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[36903.070883] usb 1-1.4: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'requires_discovery': True}, {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '1ee1818b912b6cf3ea206b30bf7d394eddd998176684122fcebd0c1a972fff1c', 'kernel_prefix_sha256': 'b3bcd8f9d90c866295aace63831c8583516095c4d39fb18a42c34445629e53cf', 'kernel_line_count': 967}, 'historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[71514.103758] usb 1-1.3: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[73274.437850] usb 1-1.3: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[78166.615277] usb 1-1.3: USB disconnect, device number 8', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[79843.245789] usb 1-1.3: USB disconnect, device number 9', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[82677.647322] usb 1-1.3: USB disconnect, device number 10', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[82987.887775] usb 1-1.4: USB disconnect, device number 11', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83012.205481] usb 1-1.4: USB disconnect, device number 12', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[83245.141873] usb 1-1.4: USB disconnect, device number 13', 'reason': 'Kernel reported a USB disconnect/reset/error'}], 'requires_discovery': True}, {'boot_id': 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '92f372030bf2b6c034f0b4a85c88c7efafb9f108c5158a534de03767797580da', 'kernel_prefix_sha256': 'e75b1b38daef8e19dfcbf81af282b40e309e8fc02cd1c23342ef759f3ace04de', 'kernel_line_count': 962}, 'historical_faults': [{'line': '[    7.573470] systemd-journald[273]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.944418] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 3008.252350] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8518.691466] usb 1-1: USB disconnect, device number 2', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8518.691521] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8518.931957] usb 2-1: USB disconnect, device number 2', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8544.480306] txrx_err:<aicwf_sdio_wakeup,1312>: sdio wakeup fail', 'reason': 'Kernel reported an SD/MMC failure'}]}, {'boot_id': 'dcf6255f-2afb-4262-8c96-4a00bc0d8cab', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '71d65dfb0a1da1c04e34054a551919f682ac460241c35456a9264bf85d8169bd', 'kernel_prefix_sha256': '1754adca02d6739e910cae4e05e9a0b995a71fc54e802e464ce541bcaef72f48', 'kernel_line_count': 963}, 'historical_faults': [{'line': '[    7.573470] systemd-journald[273]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.944418] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 3008.252350] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8518.691466] usb 1-1: USB disconnect, device number 2', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8518.691521] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8518.931957] usb 2-1: USB disconnect, device number 2', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[ 8544.480306] txrx_err:<aicwf_sdio_wakeup,1312>: sdio wakeup fail', 'reason': 'Kernel reported an SD/MMC failure'}, {'line': '[ 9890.432382] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff', 'reason': 'Feedback guard reported an AIC SDIO WiFi driver failure'}]}, {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'dd634b2e8dfe4432a70024254c6533f2f57df56043b0b9edf92793bcfa9fb529', 'kernel_prefix_sha256': '69769ba003f945aa7fc3e730b840cfb63acf312d75c2d464b45a956157e81372', 'kernel_line_count': 922}, 'historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}]}, {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'a6c3ce1cefbcf87f729accd48b549b01168d7addb5ba8f0cb2d4ce31478f309e', 'kernel_prefix_sha256': '2f71ae9a6ebdae673996e23a3b2eb29614639b3afb0569ddfc4af68cda12ae4d', 'kernel_line_count': 935}, 'historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}]}, {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': 'ffa15f5e97683645565a364d754a30b170acac8012fdec1ffc0c6995d467f127', 'kernel_prefix_sha256': 'fd6c0ba91c5b5ea92337ef83771d6e3ae4c556f260b072f1d4e25dab40aaeaeb', 'kernel_line_count': 944}, 'historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}]}, {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '7b69c8ee69665812eb2ccc76b0e202ab66df7039acdad0b80751ab5797d648b4', 'kernel_prefix_sha256': 'f0b1550b4a9e4053b176ce07244eae2c9882dec3be318653692a76b6881ed675', 'kernel_line_count': 953}, 'historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}]}, {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '069f4fc66898a5f2767c1881a5e6e2fc86434a31264a1eb43e5b41ee6f0d11d3', 'kernel_prefix_sha256': 'e0fe3dc00aa4481850bfd4d340697f2e2e89169b52e73a9f8174f5f1333c7781', 'kernel_line_count': 961}, 'historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[71514.103758] usb 1-1.3: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}]}, {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '3071e1cfab266ef8764eab9e856bc086a379425933c578ed884523b74d2da6b7', 'kernel_prefix_sha256': '5912ce0ee02430818193adf16bff3ba76729aeba8689966180adebdc9c7e20b5', 'kernel_line_count': 964}, 'historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[71514.103758] usb 1-1.3: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[73274.437850] usb 1-1.3: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}]}, {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '1ff3566694a6f2d50da3211e307cdb4e9ac9cc8e6f6b55e782636156686d232c', 'kernel_prefix_sha256': 'a90b070111c45c7eb24ad355c24d92e1e0ea9e0c766e9b976910ae523b635dbd', 'kernel_line_count': 963}, 'historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[71514.103758] usb 1-1.3: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[73274.437850] usb 1-1.3: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[78166.615277] usb 1-1.3: USB disconnect, device number 8', 'reason': 'Kernel reported a USB disconnect/reset/error'}]}, {'boot_id': '8583b8f1-2d9d-4605-b34f-facbcf18f328', 'proof': {'scope': 'feedback_only_reviewed_reconnect', 'profile_sha256': '69d3c8d2f98de3a8e8a3243cae2601db1d77f336891b37fb57d7b10a733af07f', 'kernel_prefix_sha256': 'fdcfebeb2cbadba85645ed1ecde16c7fe3f9eed82bda6cad5f11d088b36ea0c4', 'kernel_line_count': 962}, 'historical_faults': [{'line': '[    7.293643] systemd-journald[274]: File /var/log/journal/27b188d70ccc48d0a79c7117a8dd9aa3/system.journal corrupted or uncleanly shut down, renaming and replacing.', 'reason': 'Journald reported a corrupt or uncleanly closed journal file'}, {'line': '[   19.321881] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.', 'reason': 'Kernel reported an unclean FAT volume or filesystem fault'}, {'line': '[ 5127.470586] usb 1-1.3: USB disconnect, device number 3', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[68102.480531] usb 1-1.3: USB disconnect, device number 4', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[69821.346890] usb 1-1.3: USB disconnect, device number 5', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[71514.103758] usb 1-1.3: USB disconnect, device number 6', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[73274.437850] usb 1-1.3: USB disconnect, device number 7', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[78166.615277] usb 1-1.3: USB disconnect, device number 8', 'reason': 'Kernel reported a USB disconnect/reset/error'}, {'line': '[79843.245789] usb 1-1.3: USB disconnect, device number 9', 'reason': 'Kernel reported a USB disconnect/reset/error'}]})
class ProbeError(RuntimeError):
    def __init__(self, message, metadata=None):
        super().__init__(message)
        self.metadata = metadata if metadata is not None else {}

def _check_deadline(deadline):
    if time.monotonic() >= deadline:
        raise ProbeError("Overall probe deadline expired")

def _crc16(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xFFFF
    return value

def _frame(kind, sequence):
    if kind not in (1, 2):
        raise ProbeError("Only HELLO and STATE requests are permitted")
    body = HEADER.pack(b"XD", 1, kind, 0, sequence)
    return body + struct.pack("<H", _crc16(body))

def _holders(rdev, deadline):
    """Inspect proc FD metadata, never open/read another process's device FD."""
    owners = []
    vanished = (errno.ENOENT, errno.ESRCH)
    with os.scandir("/proc") as processes:
        for process in processes:
            _check_deadline(deadline)
            if not process.name.isdecimal():
                continue
            try:
                with os.scandir(process.path + "/fd") as descriptors:
                    for descriptor in descriptors:
                        _check_deadline(deadline)
                        try:
                            info = descriptor.stat(follow_symlinks=True)
                        except OSError as error:
                            if error.errno in vanished:
                                continue
                            raise ProbeError("Cannot inspect an existing process FD") from error
                        if stat.S_ISCHR(info.st_mode) and info.st_rdev == rdev:
                            owners.append({"pid": int(process.name), "fd": descriptor.name})
            except OSError as error:
                if error.errno in vanished:
                    continue
                raise ProbeError("Cannot inspect all process FD owners") from error
    return owners

def _modem_bits(fd):
    # TIOCMGET is observational. No TIOCMSET/TIOCMBIS/TIOCMBIC is used.
    result = fcntl.ioctl(fd, termios.TIOCMGET, struct.pack("I", 0))
    bits = struct.unpack("I", result)[0]
    return {"raw": bits, "dtr": bool(bits & termios.TIOCM_DTR),
            "rts": bool(bits & termios.TIOCM_RTS)}

def _raw_settings(fd):
    if not hasattr(termios, "B921600"):
        raise ProbeError("921600 baud is not supported by this termios runtime")
    original = termios.tcgetattr(fd)
    configured = list(original)
    configured[6] = list(original[6])
    configured[0] = 0
    configured[1] = 0
    configured[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB |
                       termios.HUPCL | getattr(termios, "CRTSCTS", 0))
    configured[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    configured[3] = 0
    configured[4] = configured[5] = termios.B921600
    configured[6][termios.VMIN] = 0
    configured[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, configured)
    actual = termios.tcgetattr(fd)
    blocked = termios.PARENB | termios.CSTOPB | termios.HUPCL | getattr(termios, "CRTSCTS", 0)
    if (actual[0] or actual[1] or actual[3] or actual[2] & blocked or
            actual[2] & termios.CSIZE != termios.CS8 or
            not actual[2] & termios.CREAD or not actual[2] & termios.CLOCAL or
            actual[4] != termios.B921600 or actual[5] != termios.B921600 or
            actual[6][termios.VMIN] != 0 or actual[6][termios.VTIME] != 0):
        raise ProbeError("Serial configuration readback differs from requested raw settings")
    return {"baud": 921600, "raw": True, "hupcl": False,
            "original_hupcl": bool(original[2] & termios.HUPCL),
            "settings_restored_on_close": False}

def _ready(fd, read, deadline):
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ProbeError("Response deadline expired")
        try:
            readable, writable, exceptional = select.select(
                [fd] if read else [], [] if read else [fd], [fd], remaining)
        except InterruptedError:
            continue
        if exceptional:
            raise ProbeError("Serial descriptor signaled an exception")
        if readable if read else writable:
            return

def _read_exact(fd, count, deadline):
    data = bytearray()
    while len(data) < count:
        _ready(fd, True, deadline)
        try:
            chunk = os.read(fd, count - len(data))
        except (BlockingIOError, InterruptedError):
            continue
        if not chunk:
            raise ProbeError("Serial stream ended before a complete reply")
        data.extend(chunk)
    return bytes(data)

def _write_once(fd, packet, deadline, metadata):
    # Partial writes finish this single frame; they never resend any byte/frame.
    offset = 0
    while offset < len(packet):
        _ready(fd, False, deadline)
        try:
            count = os.write(fd, packet[offset:])
        except (BlockingIOError, InterruptedError):
            continue
        if count <= 0:
            raise ProbeError("Serial request write made no progress")
        offset += count
        metadata["request_bytes_written"] += count

def _exchange(fd, kind, sequence, total_deadline, metadata):
    _check_deadline(total_deadline)
    # There is no parser recovery or input flushing: stale/noisy bytes fail closed.
    if select.select([fd], [], [], 0)[0]:
        raise ProbeError("Unexpected serial input before request")
    deadline = min(total_deadline, time.monotonic() + REPLY_SECONDS)
    metadata["request_types_attempted"].append(kind)
    _write_once(fd, _frame(kind, sequence), deadline, metadata)
    raw_header = _read_exact(fd, HEADER.size, deadline)
    magic, version, response_kind, size, response_sequence = HEADER.unpack(raw_header)
    expected_kind, expected_size = (0x81, 16) if kind == 1 else (0x82, STATE_PACKET.size)
    if (magic != b"XD" or version != 1 or response_kind != expected_kind or
            response_sequence != sequence or size != expected_size):
        raise ProbeError("Unknown or mismatched reply header")
    tail = _read_exact(fd, size + 2, deadline)
    payload, supplied_crc = tail[:-2], struct.unpack("<H", tail[-2:])[0]
    if _crc16(raw_header + payload) != supplied_crc:
        raise ProbeError("Reply CRC mismatch")
    if select.select([fd], [], [], 0)[0]:
        raise ProbeError("Unexpected trailing serial input")
    metadata["reply_types_received"].append(response_kind)
    return payload

def _decode_state(payload):
    if len(payload) != STATE_PACKET.size:
        raise ProbeError("State length mismatch")
    values = STATE_PACKET.unpack(payload)
    sample_us, imu_us, online, fault, torque, armed, imu_ok = values[:7]
    positions, velocities = list(values[7:22]), list(values[22:37])
    accel, gyro = list(values[37:40]), list(values[40:43])
    cookie, error, configured = values[43:]
    if ((online | fault | torque) & ~ALL_MASK or armed not in (0, 1) or
            imu_ok not in (0, 1) or configured not in (0, 1) or error not in range(6) or
            any(not math.isfinite(value) for value in accel + gyro) or
            any(online & (1 << index) and not 0 <= position <= 4095
                for index, position in enumerate(positions))):
        raise ProbeError("Invalid state field")
    if armed:
        raise ProbeError("Firmware reports ARMED; no further command will be sent")
    if cookie:
        raise ProbeError("Disarmed firmware reports a nonzero arm cookie")
    return {"sample_us": sample_us, "imu_us": imu_us,
            "online_mask": online, "fault_mask": fault, "torque_mask": torque,
            "armed": False, "imu_ok": bool(imu_ok), "configured": bool(configured),
            "positions": positions, "velocities": velocities,
            "accel_ms2": accel, "gyro_dps": gyro, "arm_cookie": cookie,
            "last_error": error, "motor_ids": list(IDS),
            "online_ids": [motor for index, motor in enumerate(IDS) if online & (1 << index)],
            "fault_ids": [motor for index, motor in enumerate(IDS) if fault & (1 << index)],
            "torque_ids": [motor for index, motor in enumerate(IDS) if torque & (1 << index)]}
def classify_log(line):
    """Return a stop reason for an unsafe kernel message, otherwise None."""
    # Whole-message exceptions confirmed in D-Robotics/x5-kernel sources:
    # drivers/regulator/core.c::of_get_regulator (dev_dbg, absent DT property),
    # fs/ext4/super.c (KERN_INFO after journal recovery). Do not ignore arbitrary
    # "failed"/"recovery" lines or a message with an appended fault.
    message = re.sub(r"^\s*(?:<\d+>)?(?:\[\s*\d+(?:\.\d+)?\]\s*)?", "", line)
    if re.fullmatch(r"sdhci-dwcmshc [0-9a-f]+\.sdhci: Looking up "
                    r"v(?:q)?mmc-supply property in node /\S+ failed", message):
        return None
    if re.fullmatch(r"EXT4-fs \([A-Za-z0-9_.:/-]+\): recovery complete", message):
        return None
    if re.search(r"\b(?:fat-fs|vfat|fat32)\b.*(?:not properly unmounted|unclean|"
                 r"corrupt|\berrors?\b(?!\s*=)|fail(?:ed|ure)?|read.only|please run fsck)", line, re.I):
        return "Kernel reported an unclean FAT volume or filesystem fault"
    if re.search(r"\b(?:systemd-)?journald(?:\[\d+\])?:.*(?:corrupt|\bunclean)", line, re.I):
        return "Journald reported a corrupt or uncleanly closed journal file"
    if re.search(r"\b(?:systemd-)?journald(?:\[\d+\])?:.*failed to (?:read|write) journal", line, re.I):
        return "Journald reported a journal-file read/write failure"
    if re.search(r"(?:\bi/o\s+error\b|buffer\s+i/o\s+error|blk_update_request.*i/o\s+error|"
                 r"end_request.*i/o\s+error|aborting\s+journal|"
                 r"remount(?:ing|ed)?.*read.only)", line, re.I):
        return "Kernel reported a disk I/O or journal failure"
    if re.search(r"\b(?:ext4(?:-fs)?|jbd2)\b.*(?:\berrors?\b(?!\s*=)|abort|failed|failure|"
                 r"timed?\s*out|read.only)", line, re.I):
        return "Kernel reported an EXT4/JBD2 failure"
    if (re.search(r"\b(?:mmc\w*|sdhci\w*|sdmmc\w*|sd[a-z]+|sd)\b", line, re.I)
            and re.search(r"(?:i/o|\berrors?\b(?!\s*=)|fail(?:ed|ure)?|timeout|timed?\s*out|"
                          r"\bcrc\b|recovery|\breset(?:ting)?\b|removed|removal|\bstuck\b|"
                          r"never.*\b(?:busy|programming)\b)", line, re.I)):
        return "Kernel reported an SD/MMC failure"
    if re.search(r"\bcard\b.*(?:stuck|never.*left.*busy)", line, re.I):
        return "Kernel reported a card stuck in a busy/programming state"
    if re.search(r"\busb\b.*(?:disconnect|reset.*\busb\b.*device|"
                 r"device.*(?:error|timeout|timed?\s*out))", line, re.I):
        return "Kernel reported a USB disconnect/reset/error"
    if re.search(r"\bxhci\w*\b.*(?:not responding|hc died|timeout|error)", line, re.I):
        return "Kernel reported a USB host-controller failure"
    if re.search(r"\b(?:under[- ]?voltage|brown[- ]?out)\b.*(?:detect|warn|error|fail|reset)", line, re.I):
        return "Kernel reported an unstable power supply"
    if re.search(r"(?:\b(?:dmesg|kmsg)\b.*(?:overflow|overrun|lost|too small|read.*fail)|"
                 r"messages?\s+(?:lost|dropped|suppressed)|callbacks\s+suppressed)", line, re.I):
        return "Kernel log loss prevents a reliable health check"
    return None
def additional_feedback_fault(line):
    """Extra WiFi-driver stops for feedback only; the old classifier is intact.

    A source marker and an explicit failing result identify AIC driver errors.
    A negative buffer_cnt is also a conservative stop even when its line omits
    the driver name; that alone does not identify its source. Ordinary init,
    config diagnostics, bus-down text, and a retry count alone do not establish
    a failed transfer. No matched event is ever exempted.
    """
    negative_buffers = re.finditer(r"\bbuffer_cnt\s*[:=]\s*(-\d+)\b", line, re.I)
    if any(int(match.group(1)) < 0 for match in negative_buffers):
        return "Feedback guard reported a negative driver buffer count"
    if not re.search(r"\b(?:aicwf_[a-z0-9_]+|aicsdio|aic8800[a-z0-9_]*)\b", line, re.I):
        return None
    negatives = re.finditer(r"\b(?:ret(?:urn)?|rc|errno|err(?:or)?)\s*[:=]\s*(-\d+)\b",
                           line, re.I)
    negative_result = any(int(match.group(1)) < 0 for match in negatives)
    timeout = re.search(r"\b(?:time[ -]?out|timed[ -]?out|etimedout|eio|enodev|ecomm|eilseq|eremoteio)\b",
                        line, re.I)
    transfer_failure = re.search(
        r"\b(?:sdio[ _:-]*)?(?:read|write|wakeup|wake|transfer|irq)[ _:-]+(?:fail(?:ed|ure)?|error)\b",
        line, re.I)
    severity_failure = (re.search(r"\b(?:sdio_err|txrx_err)\s*:", line, re.I)
                        and re.search(r"\bfail(?:ed|ure)?\b", line, re.I))
    if negative_result or timeout or transfer_failure or severity_failure:
        return "Feedback guard reported an AIC SDIO WiFi driver failure"
    return None

def classify_feedback_log(line):
    # Classification depends only on the exact line and immutable source.
    # Reuse that pure result, never a kernel snapshot or an acceptance proof:
    # every inventory still reads and validates the full current history.
    return classify_log(line) or additional_feedback_fault(line)

classify_feedback_log = functools.lru_cache(maxsize=2048)(classify_feedback_log)

class ReaderStopped(ProbeError):
    pass

class FeedbackEnded(Exception):
    pass

class ControlChannel:
    """One fixed termination message; no calibration or motor commands."""
    def __init__(self, fd):
        self.fd = fd
        self.buffer = bytearray()
        self.partial_started = None
        self.ended = False
    def poll(self):
        if self.ended:
            return True
        now = time.monotonic()
        if self.partial_started is not None and now-self.partial_started >= .20:
            raise ProbeError('Mac end-feedback control message did not complete within .20 seconds')
        readable, _, exceptional = select.select([self.fd], [], [self.fd], 0)
        if exceptional:
            raise ProbeError('Mac control pipe signaled an exception')
        if not readable:
            return False
        try:
            chunk = os.read(self.fd, 64-len(self.buffer))
        except BlockingIOError:
            return False
        if not chunk:
            raise ProbeError('Mac control stdin ended without an explicit END_FEEDBACK message')
        if self.partial_started is None:
            self.partial_started = now
        self.buffer.extend(chunk)
        if len(self.buffer) > 64 or not CONTROL_END.startswith(self.buffer):
            raise ProbeError('Malformed Mac control input; only END_FEEDBACK newline is allowed')
        if self.buffer == CONTROL_END:
            self.ended = True
            return True
        return False

def _signal_stop(signum, frame):
    raise ReaderStopped('Feedback reader received signal ' + str(signum))

def bounded_read(path, maximum=65536, binary=False):
    # Only proc/sys metadata is read here; no device, disk image or file writes.
    with open(path, 'rb') as stream:
        data = stream.read(maximum + 1)
    if len(data) > maximum:
        raise ProbeError('Metadata exceeds bounded size: ' + str(path))
    return data if binary else data.decode('utf-8', 'strict').strip()

def bounded_dmesg():
    child = subprocess.Popen(['dmesg', '--color=never'], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, close_fds=True)
    stdout_fd, stderr_fd = child.stdout.fileno(), child.stderr.fileno()
    buffers = {stdout_fd: bytearray(), stderr_fd: bytearray()}
    active = set(buffers)
    deadline = time.monotonic() + .30
    try:
        while active:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ProbeError('Kernel log read exceeded its bounded deadline')
            ready, _, exceptional = select.select(list(active), [], list(active), remaining)
            if exceptional:
                raise ProbeError('Kernel log pipe failed')
            for fd in ready:
                chunk = os.read(fd, 65536)
                if not chunk:
                    active.remove(fd)
                    continue
                buffers[fd].extend(chunk)
                maximum = MAX_KERNEL_BYTES if fd == stdout_fd else 8192
                if len(buffers[fd]) > maximum:
                    raise ProbeError('Kernel log or stderr exceeded its bounded size')
        status = child.wait(timeout=max(.001, deadline-time.monotonic()))
        stdout = bytes(buffers[stdout_fd])
        stderr = bytes(buffers[stderr_fd])
        if status or stderr or not stdout.strip():
            raise ProbeError('Kernel log is unavailable or reported stderr')
        return stdout.decode('utf-8', 'strict')
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=.3)
        child.stdout.close()
        child.stderr.close()

def validate_kernel(kernel, expected):
    lines = kernel.splitlines()
    # Only the unchanged, completely classified baseline can be reused. The
    # cache key also binds every historical line/reason, so changing either the
    # expected prefix or its allowances requires full classification again.
    baseline_key = (expected['kernel_prefix'],
                    tuple((item['line'], item['reason']) for item in expected['historical_faults']),
                    json.dumps(expected.get('reviewed_reconnect_proof'), sort_keys=True, allow_nan=False),
                    expected['boot_id'], expected['cid'], expected['root_mount_line'], expected['model'],
                    json.dumps(expected['serial'], sort_keys=True, allow_nan=False))
    baseline = getattr(validate_kernel, '_classified_baseline', None)
    if baseline is None or baseline['key'] != baseline_key:
        prefix = expected['kernel_prefix'].splitlines()
    else:
        prefix = baseline['lines']
    if not prefix or lines[:len(prefix)] != prefix:
        raise ProbeError('Retained kernel history changed or lost its reviewed prefix')
    if baseline is None or baseline['key'] != baseline_key:
        proof = expected.get('reviewed_reconnect_proof')
        if proof is None:
            if (expected['historical_faults'] != REVIEWED_HISTORICAL_FAULTS
                or expected['boot_id'] != REVIEWED_BOOT_ID or expected['cid'] != REVIEWED_CID):
                raise ProbeError('Default feedback requires its original boot/card and three unresolved events')
        else:
            variant = next((item for item in RECONNECT_VARIANTS if item['proof'] == proof), None)
            if (variant is None or expected['historical_faults'] != variant['historical_faults']
                or expected['boot_id'] != variant['boot_id'] or expected['cid'] != REVIEWED_CID
                or expected['root_mount_line'] != RECONNECT_ROOT or expected['model'] != RECONNECT_MODEL
                or expected['serial'] != RECONNECT_SERIAL or len(prefix) != proof['kernel_line_count']
                or hashlib.sha256(expected['kernel_prefix'].encode('utf-8')).hexdigest() != proof['kernel_prefix_sha256']):
                raise ProbeError('Feedback-only reconnect proof differs from its fixed reviewed artifact')
        first = next((float(match.group(1)) for line in prefix
                      if (match := re.match(r'^\s*\[\s*(\d+(?:\.\d+)?)\]', line))), None)
        if first is None or first > 1:
            raise ProbeError('Early kernel history is unavailable')
        faults = [{'line': line, 'reason': reason} for line in prefix
                  if (reason := classify_feedback_log(line))]
        if faults != expected['historical_faults']:
            raise ProbeError('Reviewed baseline faults changed or are incomplete')
        baseline = {'key': baseline_key, 'lines': prefix, 'faults': faults}
        validate_kernel._classified_baseline = baseline
    # Matching the entire retained prefix above is mandatory on every check.
    # Even an identical historical warning replayed in the tail is a new fault.
    tail = lines[len(prefix):]
    new_faults = [{'line': line, 'reason': reason, 'tail_index': index}
                  for index, line in enumerate(tail) if (reason := classify_feedback_log(line))]
    if new_faults:
        # Classifier labels describe a conservative stop condition, not proof
        # that the TF card caused the message. Keep the kernel source visible
        # (for example USB topology or an aicwf SDIO driver), with bounded
        # neighboring lines, rather than requiring another hardware query.
        details = []
        for fault in new_faults[:4]:
            index = fault['tail_index']
            details.append({'source': 'new_kernel_tail',
                            'kernel_line_number': len(prefix)+index+1,
                            'line': fault['line'][:512],
                            'line_truncated': len(fault['line']) > 512,
                            'classifier_reason': fault['reason'][:256],
                            'context': [{'kernel_line_number': len(prefix)+neighbor+1,
                                         'line': tail[neighbor][:512],
                                         'line_truncated': len(tail[neighbor]) > 512}
                                        for neighbor in range(max(0,index-1), min(len(tail),index+3))]})
        evidence = {'new_fault_count': len(new_faults), 'faults': details,
                    'fault_details_truncated': len(new_faults) > 4}
        raise ProbeError('New kernel fault; classifier result requires source review: '
                         + json.dumps(evidence, ensure_ascii=False, separators=(',',':')),
                         {'new_kernel_fault_evidence': evidence})
    normalized = ('\n'.join(lines)+'\n').encode('utf-8')
    return {'kernel_log': kernel, 'kernel_sha256': hashlib.sha256(normalized).hexdigest(),
            'kernel_lines': len(lines), 'historical_faults': [dict(item) for item in baseline['faults']],
            'historical_faults_cleared': False}

_SERIAL_BINDINGS = {}

def _usb_serial_identity(path):
    match = re.fullmatch(r'/dev/ttyUSB(0|[1-9][0-9]{0,2})', str(path))
    if match is None or int(match.group(1)) > 255:
        raise ProbeError('Invalid CH340 serial device path')
    info = os.stat(path, follow_symlinks=False)
    if not stat.S_ISCHR(info.st_mode) or (os.major(info.st_rdev), os.minor(info.st_rdev)) != (188, int(match.group(1))):
        raise ProbeError('CH340 character node does not match its tty number')
    tty = (Path('/sys/class/tty') / Path(path).name / 'device').resolve(strict=True)
    usb = next((parent for parent in (tty, *tty.parents)
                if (parent/'idVendor').is_file() and (parent/'idProduct').is_file()), None)
    if usb is None or re.fullmatch(r'[1-9][0-9]*-[1-9][0-9]*(?:\.[1-9][0-9]*)*', usb.name) is None:
        raise ProbeError('CH340 USB identity unavailable')
    actual = {'path': str(path), 'sysfs_name': usb.name,
              'vid': bounded_read(usb/'idVendor', 32).lower(),
              'pid': bounded_read(usb/'idProduct', 32).lower(),
              'serial_number': bounded_read(usb/'serial', 256) if (usb/'serial').is_file() else ''}
    return info, actual

def bind_serial(expected):
    """Discover once at explicit connection entry; never switch a live UART."""
    if expected.get('discover_serial') is not True:
        return expected['serial']
    key = (expected['generation'], json.dumps(expected['serial'], sort_keys=True))
    if key in _SERIAL_BINDINGS:
        serial_proof(expected)
        return dict(_SERIAL_BINDINGS[key]['identity'])
    nodes = sorted(Path('/sys/class/tty').glob('ttyUSB*'))
    if len(nodes) > 16:
        raise ProbeError('Too many USB serial devices; select ESP32 explicitly')
    candidates = []
    for node in nodes:
        try:
            info, actual = _usb_serial_identity('/dev/' + node.name)
        except (OSError, ProbeError):
            continue
        if all(actual[field] == expected['serial'][field] for field in ('vid','pid','serial_number')):
            candidates.append((info, actual))
    if not candidates:
        raise ProbeError('ESP32 USB serial device not found; check its COM cable and X5 USB connection')
    if len(candidates) != 1:
        raise ProbeError('Multiple matching CH340 USB serial devices; cannot identify ESP32 uniquely')
    info, actual = candidates[0]
    _SERIAL_BINDINGS[key] = {'identity': actual,
                            'node': (info.st_dev, info.st_ino, info.st_rdev)}
    serial_proof(expected)
    return dict(actual)

def serial_identity(expected):
    if expected.get('discover_serial') is not True:
        return expected['serial']
    key = (expected['generation'], json.dumps(expected['serial'], sort_keys=True))
    if key not in _SERIAL_BINDINGS:
        raise ProbeError('ESP32 serial discovery has not completed')
    return _SERIAL_BINDINGS[key]['identity']

def serial_path(expected):
    return serial_identity(expected)['path']

def serial_proof(expected, fd=None):
    identity = serial_identity(expected)
    if expected.get('discover_serial') is True:
        info, actual = _usb_serial_identity(identity['path'])
        key = (expected['generation'], json.dumps(expected['serial'], sort_keys=True))
        if ((info.st_dev, info.st_ino, info.st_rdev) != _SERIAL_BINDINGS[key]['node']
                or actual != identity):
            raise ProbeError('Selected ESP32 USB serial device changed; reconnect explicitly')
        if fd is not None:
            opened = os.fstat(fd)
            if (opened.st_dev, opened.st_ino, opened.st_rdev, stat.S_IFMT(opened.st_mode)) != (
                    info.st_dev, info.st_ino, info.st_rdev, stat.S_IFCHR):
                raise ProbeError('Opened UART no longer matches its physical node')
        return info, dict(actual)
    info = os.stat(identity['path'], follow_symlinks=False)
    if not stat.S_ISCHR(info.st_mode) or (os.major(info.st_rdev), os.minor(info.st_rdev)) != (188, 0):
        raise ProbeError('Reviewed ttyUSB0 character node changed')
    tty = Path('/sys/class/tty/ttyUSB0/device').resolve(strict=True)
    usb = next((parent for parent in (tty, *tty.parents)
                if (parent/'idVendor').is_file() and (parent/'idProduct').is_file()), None)
    fixed_usb = Path('/sys/bus/usb/devices/1-1.3').resolve(strict=True)
    if usb is None or usb != fixed_usb or usb.name != identity['sysfs_name']:
        raise ProbeError('Serial USB physical topology changed')
    actual = {'path': identity['path'], 'sysfs_name': usb.name,
              'vid': bounded_read(usb/'idVendor', 32).lower(),
              'pid': bounded_read(usb/'idProduct', 32).lower(),
              'serial_number': bounded_read(usb/'serial', 256) if (usb/'serial').is_file() else ''}
    if actual != identity:
        raise ProbeError('Serial USB VID/PID/serial identity changed')
    if fd is not None:
        opened = os.fstat(fd)
        if (opened.st_dev, opened.st_ino, opened.st_rdev, stat.S_IFMT(opened.st_mode)) != (
                info.st_dev, info.st_ino, info.st_rdev, stat.S_IFCHR):
            raise ProbeError('Opened UART no longer matches its physical node')
    return info, actual

def validate_os(metadata, expected):
    if (metadata['boot_id'] != expected['boot_id'] or metadata['cid'] != expected['cid']
            or metadata['root_mount_line'] != expected['root_mount_line']
            or metadata['model'] != expected['model']):
        raise ProbeError('Reviewed boot/root/card/model identity changed')
    swaps = metadata['swaps'].splitlines()
    if not swaps or swaps[0].split() != ['Filename','Type','Size','Used','Priority'] or len(swaps) != 1:
        raise ProbeError('Swap is active or its inventory is malformed')
    root = metadata['root_mount_line'].split()
    try:
        separator = root.index('-', 6)
        writable = ('rw' in root[5].split(',') and 'ro' not in root[5].split(',')
                    and 'ro' not in root[separator+3].split(','))
    except (ValueError, IndexError):
        writable = False
    if not writable:
        raise ProbeError('Root filesystem is no longer writable')

def health_snapshot(expected, fd=None):
    mountinfo = bounded_read('/proc/self/mountinfo', 262144)
    roots = [line for line in mountinfo.splitlines()
             if len(line.split()) > 4 and line.split()[4] == '/']
    if len(roots) != 1:
        raise ProbeError('Root mount is not unique')
    metadata = {'boot_id': bounded_read('/proc/sys/kernel/random/boot_id', 128),
                'cid': bounded_read('/sys/block/mmcblk1/device/cid', 128),
                'root_mount_line': roots[0],
                'model': bounded_read('/proc/device-tree/model', 4096, binary=True).rstrip(b'\0').decode('utf-8','strict'),
                'swaps': bounded_read('/proc/swaps', 16384)}
    validate_os(metadata, expected)
    node = os.stat('/dev/mmcblk1p2', follow_symlinks=False)
    if not stat.S_ISBLK(node.st_mode) or (os.major(node.st_rdev), os.minor(node.st_rdev)) != (179,2):
        raise ProbeError('Canonical root block-node identity changed')
    if bounded_read('/sys/block/mmcblk1/device/type', 32) != 'SD':
        raise ProbeError('Root card is no longer an SD card')
    _, metadata['serial'] = serial_proof(expected, fd)
    metadata.update(validate_kernel(bounded_dmesg(), expected))
    return metadata

class StateTracker:
    def __init__(self):
        self.previous = None
        self.sample_progress = self.imu_progress = None
    def accept(self, state, now):
        if state['armed'] is not False or state['arm_cookie'] or state['fault_mask'] or state['torque_mask']:
            raise ProbeError('Unsafe armed/cookie/fault/torque feedback; reader will close')
        if state['online_mask'] not in (0, ALL_MASK):
            raise ProbeError('Partial servo communication; reader will close')
        if state['last_error'] or state['imu_ok'] is not True:
            raise ProbeError('Firmware error or invalid IMU feedback')
        if any(not math.isfinite(value) for value in state['accel_ms2'] + state['gyro_dps']):
            raise ProbeError('Non-finite IMU data')
        age = (state['sample_us'] - state['imu_us']) & 0xffffffff
        if 500000 < age < 0x80000000:
            raise ProbeError('IMU sample is stale relative to servo sample time')
        if age >= 0x80000000 and ((state['imu_us']-state['sample_us']) & 0xffffffff) > 500000:
            raise ProbeError('IMU and servo timestamps are inconsistent')
        if self.previous is None:
            self.previous = state
            self.sample_progress = self.imu_progress = now
            return True
        advances = []
        for key, clock_key in (('sample_us','sample_progress'), ('imu_us','imu_progress')):
            delta = (state[key]-self.previous[key]) & 0xffffffff
            if delta >= 0x80000000 or delta > 500000:
                raise ProbeError('Feedback sample time reversed/reset or jumped')
            if delta:
                setattr(self, clock_key, now)
            elif now-getattr(self, clock_key) >= .5:
                raise ProbeError('Feedback timestamp stopped advancing')
            advances.append(delta > 0)
        # A read near a 20-ms MCU boundary may repeat one sample. Do not emit it
        # as fresh data or collect it for calibration; bound a frozen source.
        if not all(advances):
            return False
        self.previous = state
        return True

def emit(kind, expected, **values):
    event = {'kind': kind, 'generation': expected['generation'], **values}
    payload = (json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(',',':'))+'\n').encode('utf-8')
    if len(payload) > 3*MAX_KERNEL_BYTES:
        raise ProbeError('JSON output exceeds its bounded size')
    deadline = time.monotonic()+.20
    offset = 0
    while offset < len(payload):
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            raise ProbeError('Mac stdout pipe stopped accepting feedback')
        _, writable, exceptional = select.select([], [sys.stdout.fileno()], [sys.stdout.fileno()], remaining)
        if exceptional:
            raise ProbeError('Mac stdout pipe failed')
        if writable:
            try:
                count = os.write(sys.stdout.fileno(), payload[offset:])
            except BlockingIOError:
                continue
            if count <= 0:
                raise ProbeError('Mac stdout pipe closed')
            offset += count

def _feedback_summary(payload, sequence):
    # _exchange verified header/sequence/CRC. This independent scalar-only
    # diagnostic does not change _decode_state or StateTracker acceptance.
    # Never include positions/vectors, even when their values are malformed.
    if len(payload) != STATE_PACKET.size:
        return None, 'state_length_mismatch'
    values = STATE_PACKET.unpack(payload)
    sample_us, imu_us, online, fault, torque, armed, imu_ok = values[:7]
    cookie, error = values[43:45]
    if ((online | fault | torque) & ~ALL_MASK or armed not in (0,1)
            or imu_ok not in (0,1) or error not in range(6)):
        return None, 'invalid_scalar_fields'
    return {'sequence': sequence, 'sample_us': sample_us, 'imu_us': imu_us,
            'online_mask': online, 'fault_mask': fault, 'torque_mask': torque,
            'armed': bool(armed), 'arm_cookie': cookie, 'imu_ok': bool(imu_ok),
            'last_error': error}, None

def _query_summary(metadata):
    summary = {**metadata, 'request_types_attempted': list(metadata['request_types_attempted']),
               'reply_types_received': list(metadata['reply_types_received'])}
    if 'last_feedback' in metadata:
        summary['last_feedback'] = dict(metadata['last_feedback'])
    return summary

def _hello_feedback(fd, metadata):
    try:
        reply = _exchange(fd, 1, 1, time.monotonic()+REPLY_SECONDS, metadata)
    except ProbeError as error:
        if str(error) == 'Response deadline expired':
            raise ProbeError('ESP32 handshake response timed out') from error
        raise
    if reply != b'XGODUCK_MOTION_1':
        raise ProbeError('Unknown motion firmware identity')

def run_reader(expected):
    # An explicitly launched process gets one UART open, one HELLO, and a
    # STATE stream until its fixed end message. Every I/O and health operation
    # stays bounded. There is no STOP, reset, flushing or reconnect path.
    started = time.monotonic()
    parent = os.getppid()
    fd = None
    closed = False
    reason = 'Feedback reader stopped before normal termination'
    normal = False
    health_watch = None
    metadata = {'request_types_attempted': collections.deque(maxlen=4),
                'reply_types_received': collections.deque(maxlen=4),
                'request_bytes_written': 0, 'state_requests': 0,
                'state_records': 0, 'serial_open_attempts': 0,
                'exclusive': False, 'serial_closed': False,
                'board_files_written_by_reader': False,
                'mcu_reset_possible': True, 'electrical_safety_verified': False,
                'modem_lines_actively_changed': False,
                'firmware_binary_hash_verified': False,
                'firmware_may_autonomously_write_disable_or_imu_config': True,
                'servo_motion_authorized': False, 'storage_health_certified': False}
    tracker = StateTracker()
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, _signal_stop)
    try:
        os.set_blocking(sys.stdout.fileno(), False)
        os.set_blocking(sys.stdin.fileno(), False)
        control = ControlChannel(sys.stdin.fileno())
        def check_control():
            if os.getppid() != parent:
                raise ProbeError('Reader parent process disappeared')
            if control.poll():
                raise FeedbackEnded('Feedback ended by Mac')
        check_control()
        bind_serial(expected)
        proof = health_snapshot(expected)
        node, _ = serial_proof(expected)
        owners = _holders(node.st_rdev, time.monotonic()+.20)
        if owners:
            raise ProbeError('UART already has an open holder')
        # The /proc owner scan is bounded, and health is checked again directly
        # before the sole open rather than relying on an earlier snapshot.
        proof = health_snapshot(expected)
        check_control()
        metadata['serial_open_attempts'] += 1
        fd = os.open(serial_path(expected), os.O_RDWR | os.O_NOCTTY |
                     os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
        serial_proof(expected, fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        metadata['exclusive'] = True
        owners = [owner for owner in _holders(node.st_rdev, time.monotonic()+.20)
                  if not (owner['pid'] == os.getpid() and owner['fd'] == str(fd))]
        if owners:
            raise ProbeError('Another UART holder appeared while opening')
        metadata['modem_before_raw'] = _modem_bits(fd)
        metadata['serial_settings'] = _raw_settings(fd)
        metadata['modem_after_raw'] = _modem_bits(fd)
        if any(metadata['modem_after_raw'][line] != metadata['modem_before_raw'][line]
               for line in ('dtr','rts')):
            raise ProbeError('Observed modem signals changed during UART configuration')
        proof = health_snapshot(expected, fd)
        check_control()
        # Do not flush stale input, swallow boot logs, pulse DTR/RTS or retry.
        _hello_feedback(fd, metadata)
        proof = health_snapshot(expected, fd)
        check_control()
        emit('ready', expected, identity=proof, protocol_identity='XGODUCK_MOTION_1',
             query=_query_summary(metadata), online_requirement='zero_or_all_15',
             allowed_requests=['HELLO','STATE'], emitted_monotonic=time.monotonic(),
             reader_elapsed_seconds=time.monotonic()-started)
        last_health = time.monotonic()
        last_health_started = last_health
        # Native X5 supplies an independently monitored health worker. The
        # legacy reader keeps its synchronous health path unchanged.
        health_factory = globals().get('reader_health_watcher')
        if health_factory is not None:
            health_watch = health_factory(expected, fd)
        sequence = 1
        while True:
            tick = time.monotonic()
            check_control()
            if health_watch is not None:
                last_health = health_watch.check()
            elif tick-last_health_started > .5:
                raise ProbeError('Storage/USB health checks lost their timing guarantee')
            if health_watch is None and tick-last_health_started >= HEALTH_INTERVAL:
                last_health_started = tick
                health_snapshot(expected, fd)
                last_health = time.monotonic()
                if last_health-last_health_started > .5:
                    raise ProbeError('Storage/USB health check took too long')
                check_control()
            sequence += 1
            if sequence > 0xffffffff:
                raise ProbeError('Protocol sequence exhausted; reader will close')
            metadata.pop('last_feedback', None)
            metadata.pop('last_feedback_invalid', None)
            metadata['state_requests'] += 1
            payload = _exchange(fd, 2, sequence, time.monotonic()+REPLY_SECONDS, metadata)
            diagnostic, invalid = _feedback_summary(payload, sequence)
            if invalid is not None:
                metadata['last_feedback_invalid'] = invalid
            else:
                metadata['last_feedback'] = diagnostic
            state = _decode_state(payload)
            now = time.monotonic()
            accepted = tracker.accept(state, now)
            check_control()
            if accepted:
                metadata['state_records'] += 1
                age = (state['sample_us']-state['imu_us']) & 0xffffffff
                age_ms = age/1000 if age < 0x80000000 else 0.
                emit('state', expected, sequence=sequence, state=state,
                     received_monotonic=now, reader_elapsed_seconds=now-started,
                     age_ms=0., imu_age_ms=age_ms,
                     servo_power_observed='none_online' if state['online_mask']==0 else 'all_online',
                     health_age_ms=(now-last_health)*1000)
            delay = max(0., PERIOD_SECONDS-(time.monotonic()-tick))
            if delay:
                time.sleep(delay)
    except FeedbackEnded:
        reason = 'Feedback ended by Mac'
        normal = True
    except BaseException as error:
        reason = str(error) or type(error).__name__
        try:
            emit('fault', expected, reason=reason, query=_query_summary(metadata))
        except BaseException:
            pass  # The broken pipe itself is sufficient to end the session.
    finally:
        if health_watch is not None:
            try:
                health_watch.close()
            except BaseException as error:
                reason = 'Health worker close failed: '+str(error)
                normal = False
        if fd is not None:
            try:
                os.close(fd)
                closed = True
            except OSError as error:
                reason = 'Serial close failed: '+str(error)
                normal = False
        else:
            closed = True  # Proof means no descriptor was opened or it closed.
        metadata['serial_closed'] = closed
        try:
            emit('closed', expected, reason=reason, normal=normal,
                 serial_closed=closed, query=_query_summary(metadata),
                 elapsed_seconds=time.monotonic()-started)
        except BaseException:
            pass
    return 0 if normal and closed else 20

def run_rx_capture(expected):
    """One guarded diagnostic open: RX only, no request or recovery attempt."""
    started = time.monotonic()
    parent = os.getppid()
    fd = None
    closed = normal = truncated = False
    reason = 'RX capture stopped before completion'
    received = bytearray()
    metadata = {'request_types_attempted': collections.deque(maxlen=4),
                'reply_types_received': collections.deque(maxlen=4),
                'request_bytes_written': 0, 'serial_open_attempts': 0,
                'rx_bytes_read': 0, 'readonly_uart': True,
                'exclusive': False, 'serial_closed': False,
                'board_files_written_by_reader': False,
                'mcu_reset_possible': True, 'electrical_safety_verified': False,
                'modem_lines_actively_changed': False,
                'servo_motion_authorized': False, 'storage_health_certified': False}
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, _signal_stop)
    try:
        output_fd = sys.stdout.fileno()
        os.set_blocking(output_fd, False)
        output_poll = select.poll()
        output_poll.register(output_fd, select.POLLOUT | select.POLLERR | select.POLLHUP | select.POLLNVAL)
        def liveness():
            if os.getppid() != parent:
                raise ProbeError('RX capture parent process disappeared')
            events = output_poll.poll(0)
            if any(mask & (select.POLLERR | select.POLLHUP | select.POLLNVAL) for _, mask in events):
                raise ProbeError('RX capture Mac stdout pipe closed')
            if not any(mask & select.POLLOUT for _, mask in events):
                raise ProbeError('RX capture Mac stdout pipe is blocked')
        liveness()
        bind_serial(expected)
        proof = health_snapshot(expected)
        node, _ = serial_proof(expected)
        if _holders(node.st_rdev, time.monotonic()+.20):
            raise ProbeError('UART already has an open holder')
        proof = health_snapshot(expected)
        liveness()
        metadata['serial_open_attempts'] += 1
        fd = os.open(serial_path(expected), os.O_RDONLY | os.O_NOCTTY |
                     os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
        serial_proof(expected, fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        metadata['exclusive'] = True
        others = [owner for owner in _holders(node.st_rdev, time.monotonic()+.20)
                  if not (owner['pid'] == os.getpid() and owner['fd'] == str(fd))]
        if others:
            raise ProbeError('Another UART holder appeared while opening')
        metadata['modem_before_raw'] = _modem_bits(fd)
        metadata['serial_settings'] = _raw_settings(fd)
        metadata['modem_after_raw'] = _modem_bits(fd)
        if any(metadata['modem_after_raw'][line] != metadata['modem_before_raw'][line]
               for line in ('dtr','rts')):
            raise ProbeError('Observed modem signals changed during RX configuration')
        proof = health_snapshot(expected, fd)
        liveness()
        emit('rx_ready', expected, identity=proof, query=_query_summary(metadata),
             allowed_requests=[], capture_limits={'seconds': 1., 'bytes': 4096,
                                                   'quiet_seconds': .1, 'poll_seconds': .05})
        capture_started = last_rx = last_health_started = time.monotonic()
        deadline = capture_started + 1.
        while True:
            liveness()
            now = time.monotonic()
            if now >= deadline:
                reason = 'RX capture time limit reached'
                break
            if now-last_rx >= .1:
                reason = 'RX capture quiet interval reached'
                break
            if now-last_health_started > .5:
                raise ProbeError('RX capture lost its health timing guarantee')
            if now-last_health_started >= HEALTH_INTERVAL:
                last_health_started = now
                health_snapshot(expected, fd)
                if time.monotonic()-last_health_started > .5:
                    raise ProbeError('RX capture health check took too long')
                liveness()
                continue
            wait = min(.05, deadline-now, .1-(now-last_rx),
                       max(.001, HEALTH_INTERVAL-(now-last_health_started)))
            readable, _, exceptional = select.select([fd], [], [fd], wait)
            liveness()
            if exceptional:
                raise ProbeError('RX capture UART signaled an exception')
            if not readable:
                continue
            try:
                chunk = os.read(fd, 4096-len(received))
            except BlockingIOError:
                continue
            if not chunk:
                raise ProbeError('RX capture UART returned EOF')
            received.extend(chunk)
            metadata['rx_bytes_read'] = len(received)
            last_rx = time.monotonic()
            if len(received) >= 4096:
                truncated = True
                reason = 'RX capture byte limit reached; additional input was not consumed'
                break
        liveness()
        health_snapshot(expected, fd)
        liveness()
        normal = not truncated
    except BaseException as error:
        reason = str(error) or type(error).__name__
        try:
            emit('fault', expected, reason=reason, query=_query_summary(metadata))
        except BaseException:
            pass
    finally:
        if fd is not None:
            try:
                os.close(fd)
                closed = True
            except OSError as error:
                reason = 'Serial close failed: '+str(error)
                normal = False
        else:
            closed = True
        metadata['serial_closed'] = closed
        try:
            emit('rx_capture', expected, raw_base64=base64.b64encode(received).decode('ascii'),
                 sha256=hashlib.sha256(received).hexdigest(), count=len(received),
                 truncated=truncated, empty=not received, reason=reason,
                 elapsed_seconds=time.monotonic()-started, query=_query_summary(metadata))
            emit('closed', expected, reason=reason, normal=normal, serial_closed=closed,
                 query=_query_summary(metadata), elapsed_seconds=time.monotonic()-started)
        except BaseException:
            normal = False
    return 0 if normal and closed else 20
