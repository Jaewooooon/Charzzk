import { atom } from 'recoil';

// 배터리 상태를 저장하기 위한 atom 생성
export const batteryState = atom({
  key: 'batteryState', // 고유 ID
  default: 0, // 기본값 (초기값)
});
