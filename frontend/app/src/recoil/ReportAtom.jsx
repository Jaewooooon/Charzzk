import { atom } from 'recoil';

// 신고 상태 리코일 아톰 정의
export const reportState = atom({
  key: 'reportState',
  default: {
    serialNumber: '',
    type: '',
    content: '',
    image: null,
  },
});
