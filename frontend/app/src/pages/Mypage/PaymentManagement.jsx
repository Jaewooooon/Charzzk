import React from 'react';
import { useRecoilValue } from 'recoil';
import { accessTokenState } from '../../recoil/LoginAtom';

function PaymentManagement() {
  const accessToken = useRecoilValue(accessTokenState);
  return (
    <div>
      <h1>Access Token: {accessToken}</h1>
    </div>
  );
}

export default PaymentManagement;
