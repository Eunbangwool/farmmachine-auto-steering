# FMSC v1 — 작업기 살포 제어 CAN 프로토콜 (자체 규약)

> 가변시비(VRA)·방제 섹션 컨트롤러용 **자체 정의** CAN 프로토콜.
> 오너가 직접 만드는 작업기 컨트롤러(살포기 밸브/모터 구동 MCU)와 앱 사이 규약.
> 구현: `auto-steering/src/section_control.py`. (Keya 조향모터와 무관 — 별개 노드.)

## 왜 자체 규약인가
조향모터(Keya)는 기존 매뉴얼을 역추출했지만, 살포기 컨트롤러는 **오너가 직접 제작**한다.
따라서 상용 ISOBUS TC 를 흉내내지 않고, MCU 구현 부담이 작은 단순 고정프레임 규약을
우리가 정한다. 상용 컨트롤러를 쓰게 되면 그 프로토콜로 어댑터를 교체하면 된다.

## 버스
- **250 kbps, 29-bit 확장 ID.** Keya(0x05/0x06/0x07 대역)와 ID 가 겹치지 않으므로
  동일 물리 CAN 버스 공유 가능(또는 살포기 전용 버스 분리도 가능).

## 프레임

### APP → 컨트롤러
| 이름 | ID | 주기 | 내용 |
|---|---|---|---|
| `RATE_CMD`    | `0x0A100001` | 100 ms | 목표 살포율 + 단위 + 지면속도(피드포워드) |
| `SECTION_CMD` | `0x0A100002` | 100 ms | 섹션 ON/OFF 비트마스크 + 마스터 |

### 컨트롤러 → APP
| 이름 | ID | 주기 | 내용 |
|---|---|---|---|
| `STATUS_HB` | `0x0A180001` | 100 ms | 실제 섹션상태 + 실제 살포율 + 결함 + 호퍼레벨 |

`STATUS_HB` 가 `STATUS_TIMEOUT`(500 ms) 이상 없으면 `comm_lost`.

## 페이로드 (8바이트, 빅엔디안, b7=체크섬)

체크섬 `b7 = XOR(b0..b6)` (단순·결정적, MCU 구현 부담 최소).

### RATE_CMD (0x0A100001)
| 바이트 | 필드 | 비고 |
|---|---|---|
| b0 | 플래그 | bit0=master_on, bit1=rate_valid, bit4..7=mode(0=입제시비 1=액제방제 2=파종) |
| b1 | unit | 0=kg/ha, 1=L/ha, 2=seeds/m² |
| b2..b3 | target_rate | uint16, **scale 0.1** (값 = rate×10), 0..6553.5 |
| b4..b5 | ground_speed | uint16, mm/s (컨트롤러 유량 피드포워드용) |
| b6 | section_count | 섹션 수 |
| b7 | checksum | XOR(b0..b6) |

### SECTION_CMD (0x0A100002)
| 바이트 | 필드 | 비고 |
|---|---|---|
| b0..b1 | section_mask | uint16, **bit0=섹션1(좌측 끝)** … 좌→우 |
| b2 | section_count | |
| b3 | 플래그 | bit0=master_on |
| b4..b5 | applied_rate | uint16, scale 0.1 (실제 명령 살포율 에코) |
| b6 | reserved | 0 |
| b7 | checksum | |

### STATUS_HB (0x0A180001, 컨트롤러→앱)
| 바이트 | 필드 | 비고 |
|---|---|---|
| b0..b1 | section_mask | 실제 섹션 상태 |
| b2..b3 | actual_rate | uint16, scale 0.1 |
| b4 | faults | bit0=호퍼부족 bit1=밸브결함 bit2=과다살포 bit3=통신두절 |
| b5 | hopper_pct | 호퍼/탱크 잔량 % (0..100) |
| b6 | reserved | |
| b7 | checksum | |

## 섹션 OFF 규칙 (`SectionController`)
섹션 지면접점을 밸브 ON 선행(`lead_on_s`, 기본 0.4s)만큼 전방 투영해 판정:
1. `master_off` — 마스터 스위치 OFF
2. `headland` — 작업기 들림(회전 구간)
3. `out_of_field` — 포장 경계 밖
4. `exclusion` — 제외구역 안
5. `rate_zero` — 처방 살포율 0
6. `overlap` — 이미 살포된 영역(커버리지 그리드) → 중복 방지

## ★ 안전장치 (`IMPLEMENT_CAN_VERIFIED`)
실 컨트롤러 HW 가 없는 동안 `section_control.IMPLEMENT_CAN_VERIFIED = False`.
- False: 프레임 생성·로그·SITL 시각화는 동작, **버스 송신만 차단**(추측 송신 방지).
  `vendor_profiles.can_verified`(조향모터)와 동일 철학.
- 실 컨트롤러로 버스 캡처/검증 후 `True` 로.

## 처방맵 (GeoJSON Rx)
표준 `FeatureCollection`, 각 Feature=구역 폴리곤 + `properties.rate`(목표 살포율).
좌표는 `[lon, lat]`(GeoJSON 표준). 단위는 `properties.unit` 또는 적재 인자.
구역 밖은 `default_rate`. 차체 로컬 좌표계와 정합되도록 **autosteer RTK 원점과 동일 원점**
으로 변환(앱은 RTK Fix 확정 후 자동 적재).

## 법적 준수 (약관 §8)
살포 명령(마스크·목표/실제율·속도)을 `ApplicationController.log` 에 로컬 기록 유지.

## 호출 표면 (Kotlin/JS)
`load_prescription(geojson, default_rate, mode, unit)` · `set_implement_layout(width_m, sections, impl_behind)` ·
`set_application_master(on)` · `clear_coverage()` · `application_status()`.
상태는 `status_json().application` 에도 매 틱 포함.
