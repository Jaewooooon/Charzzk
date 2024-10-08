import React from 'react';
import { useRecoilValue } from 'recoil';
import { accessTokenState } from '../../recoil/LoginAtom';
import '../../styles/UserManagement.css';

function UserManagement() {
  const accessToken = useRecoilValue(accessTokenState);
  return (
    <div>
      <div className='UserPatch_box'>
        <h3 className='Hi_contents'>강효린님, 안녕하세요!</h3>
        </div>

    </div>
  );
}

export default UserManagement;
