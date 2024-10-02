import { atom } from 'recoil';

export const carTypesState = atom({
    key: 'carTypesState', // 고유한 키
    default: [], // 초기값
});

export const selectedCarTypeState = atom({
    key: 'selectedCarTypeState',
    default: '', // 선택한 차량 기종 (ID 저장)
});

export const nicknameState = atom({
    key: 'nicknameState',
    default: '', // 닉네임 초기값
});

export const nicknameCheckMessageState = atom({
    key: 'nicknameCheckMessageState',
    default: '', // 닉네임 중복 확인 메시지 초기값
});

export const isNicknameAvailableState = atom({
    key: 'isNicknameAvailableState',
    default: null, // 닉네임 사용 가능 여부 초기값
});

export const carNumberState = atom({
    key: 'carNumberState',
    default: '', // 차량 번호 초기값
});

export const carNicknameState = atom({
    key: 'carNicknameState',
    default: '', // 차량 별명 초기값
});