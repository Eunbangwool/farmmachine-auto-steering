package com.farmmachine.autosteer.safety

/**
 * 모터 폴트 비트필드 관리 — CHCNAV `.so` 패턴 미러.
 *
 * ★ 출처: repo 루트 `CHCNAV_PARAM_PROFILE.md` 부록(로그 검증) + §2.
 *   .so 심볼 HUACE_CheckIfFault / GetErrorCode / ClearError 의 동작 사실만 자체 구현(clean-room).
 *
 * ★ 심각 폴트 사례(PROFILE 부록, 06-04 08:03):
 *   col11=520(bit3+9), col12=0x3f00000000(bit32~37 연속) = 모터 드라이버 다중 폴트.
 *   → "연속 다중비트 동시 set" 을 심각 폴트로 판정해 즉시 disengage.
 *
 * ⚠ 개별 비트의 의미(어느 비트가 과전류/과열/홀에러인지)는 PROFILE.md 에 매핑이 없음(TODO).
 *   따라서 의미 기반 판정 대신 '연속 다중비트' 구조적 판정만 한다(임의 비트정의 하드코딩 금지).
 */
class MotorFault {

    @Volatile
    var code: Long = 0L
        private set

    /** GetErrorCode 패턴 — 현재 폴트 비트필드 반환. */
    fun getErrorCode(): Long = code

    /** ClearError 패턴 — 폴트 비트 클리어. */
    fun clearError() { code = 0L }

    /** 드라이버에서 읽은 폴트 비트필드 갱신(예: 하트비트의 폴트코드). */
    fun update(faultCode: Long) { code = faultCode }

    /** HUACE_CheckIfFault 패턴 — 폴트 비트가 하나라도 있으면 true. */
    fun checkIfFault(): Boolean = code != 0L

    /**
     * 심각 폴트 판정 — 즉시 disengage 대상.
     * PROFILE 부록 사례(bit32~37 연속) 기준: 연속 set 비트가 [threshold] 개 이상이면 심각.
     * @param threshold 연속 비트 개수 임계(기본 3 — 단발 경고와 드라이버 다중폴트 구분).
     */
    fun isSevere(threshold: Int = SEVERE_CONSECUTIVE_BITS): Boolean =
        maxConsecutiveBits(code) >= threshold

    companion object {
        /**
         * 심각 폴트로 볼 '연속 set 비트' 최소 개수.
         * PROFILE 사례는 bit32~37 = 6연속. 다중폴트(>=3연속)를 심각으로 본다.
         * ⚠ 정확한 임계는 PROFILE.md 에 수치 명시 없음 — 사례(6연속)보다 보수적인 3 으로 둔 구조적 판정.
         */
        const val SEVERE_CONSECUTIVE_BITS = 3

        /** 비트필드에서 '연속으로 set 된 1비트' 최대 길이. */
        fun maxConsecutiveBits(bits: Long): Int {
            var run = 0
            var best = 0
            for (i in 0 until 64) {
                if ((bits ushr i) and 1L == 1L) {
                    run++
                    if (run > best) best = run
                } else {
                    run = 0
                }
            }
            return best
        }
    }
}
