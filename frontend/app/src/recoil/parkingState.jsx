import { atom } from 'recoil';

export const parkingState = atom({
  key: 'parkingState', // unique ID (with respect to other atoms/selectors)
  default: {
    parkingLotId: null,
    parkingSpotId: null,
    carId: null,
    fullCharge: false,
    time: 0,
  },
});

