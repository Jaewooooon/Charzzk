import React from 'react';
import CharZzk_Logo from '../assets/WebLogo.png';
import '../styles/Charge_Start.css';

const Login = () => {
  return (
    <div className='ChageStart_Box'>
      <img src={CharZzk_Logo} alt="Car Sample" className="CharZzk_Logo" />
      <div className='start_button'>
        <button 
          className='kakao_button'
          onClick={() => {
            window.location.href = 'https://j11c208.p.ssafy.io/oauth2/authorization/kakao?redirect_uri=https://j11c208.p.ssafy.io'
          }}
        >카카오로 시작하기</button>
        
        <button className='google_button'>구글로 시작하기</button>
      </div>
    </div>
  );
}

export default Login;
