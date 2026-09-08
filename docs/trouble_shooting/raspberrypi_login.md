# [TurtleBot3] 라즈베리파이 SSH 로그인 실패 및 Windows 파일 접근 불가

- **Issue:** 중고 라즈베리파이 SSH 비밀번호 불일치로 접속 불가. 비밀번호 강제 초기화를 위해 SD카드를 Windows PC에 연결했으나 `/etc` 폴더가 보이지 않아 접근 실패. (캘리브레이션 데이터 보존을 위해 포맷 불가)

- **Cause:** 1. **접속 불가:** 판매자가 제공한 패스워드 정보 오류.
  2. **Windows 접근 실패 원인:** 라즈베리파이 OS(Ubuntu)의 핵심 파일들이 있는 파티션은 Linux 전용 파일 시스템인 `ext4`로 포맷되어 있음. Windows는 기본적으로 이를 인식하지 못하기 때문에, FAT32로 포맷된 좁은 용량의 `boot` 파티션만 보이고 정작 중요한 `/etc` 디렉토리가 있는 `rootfs` 파티션은 읽을 수 없었던 것.

- **Fix:** SD카드를 Windows가 아닌 **Ubuntu 환경**에 마운트한 뒤, `shadow` 파일의 비밀번호 해시를 직접 삭제하여 강제 초기화.

  ```bash
  # 1. Ubuntu PC에 SD카드 연결 후 rootfs 파티션의 shadow 파일 열기
  sudo vim /media/$USER/rootfs/etc/shadow
  
  # 2. ubuntu(또는 root) 계정의 패스워드 해시 부분만 삭제 후 저장
  # 변경 전: ubuntu:$6$xyz...:19000:0:99999:7:::
  # 변경 후: ubuntu::19000:0:99999:7:::