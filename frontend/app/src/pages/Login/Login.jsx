import React, { useEffect } from 'react'; 
import CharZzk_Logo from '../../assets/WebLogo.png';
import '../../styles/Login.css';
import { useRecoilState } from 'recoil';
import { accessTokenState } from '../../recoil/LoginAtom';
import { useLocation, useNavigate } from 'react-router-dom';
import GoogleLogo from '../../assets/google_logo.png';
import KakaoLogo from '../../assets/kakao_logo.png';

const Login = () => {
  const [accessToken, setAccessToken] = useRecoilState(accessTokenState);
  const location = useLocation();
  const navigate = useNavigate();



  return (
    <div className='ChageStart_Box'>
      <img src={CharZzk_Logo} alt="Car Sample" className="CharZzk_Logo" />
      <div className='start_button'>
        <button 
          className='kakao_button'
          onClick={() => {
            //나중에 url 바꾸기
            window.location.href = 'https://j11c208.p.ssafy.io/oauth2/authorization/kakao?redirect_uri=http://localhost:5173'
          }}
        >
            <img src={KakaoLogo} alt="카카오" className='kakao_logo'/>카카오로 시작하기</button>
        
        {/* 구글 로그인 버튼 */}
        <button 
          className='google_button'
          onClick={() => {
            window.location.href = 'https://j11c208.p.ssafy.io/oauth2/authorization/google?redirect_uri=https://j11c208.p.ssafy.io'
          }}
        >
            <img src={GoogleLogo} alt="구글" className='google_logo'/>구글로 시작하기</button>
      </div>
    </div>
  );
}

export default Login;
