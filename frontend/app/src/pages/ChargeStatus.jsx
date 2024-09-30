import React, { useState } from 'react';
import '../styles/ChargeStatus.css';
import GoBackButton from '../components/GobackButton';

const ChargeStatus= () => {
  const [chargePercent, setChargePercent] = useState(63); // 초기값은 40으로 설정합니다.


  return (
    <div className="ChargeStatus_ContainerBox">
      <GoBackButton />
 
      <div className="Percent_Contents">
        <p className='AutoCharge_Contents'> AutoCharge!</p>
        <p className='ChargePercent_Contents'>{chargePercent}%</p>

        <p className='ChargeStatus_Contents'>23분 후 충전 시작</p>
        <p className='ChargeStart_Contents'>충전 시작 시각 20:30 예정</p>
        <p className='ChargeComplete_Contents'>충전 완료 시각 21:42 예정</p>
        </div>
        

      <div className='Percent_Show1'></div>
      <div className='Percent_Show2'></div>
      <div className='Percent_Show3'></div>
      <div className='Percent_Show4'></div>

      
    </div>
  );
};

export default ChargeStatus;
