import { atom } from 'recoil';

export const buttonState = atom({
  key: 'buttonState', // 고유 키
  default: false, // 초기 값
});
