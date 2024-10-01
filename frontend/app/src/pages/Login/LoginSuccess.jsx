import React, { useEffect } from 'react'; 
import { useRecoilState, useSetRecoilState } from 'recoil';
import { accessTokenState } from '../../recoil/LoginAtom';
import { useLocation, useNavigate } from 'react-router-dom';
import { authState } from '../../recoil/authState';

const LoginSuccess = () => {
    const [accessToken, setAccessToken] = useRecoilState(accessTokenState);
    const location = useLocation();
    const navigate = useNavigate();
    const setAuthState = useSetRecoilState(authState);

    useEffect(() => {
      const params = new URLSearchParams(location.search);
      const token = params.get('accessToken');
      console.log('Token:', token);
      
      if (token) {
        console.log('Setting access token and auth state');
        setAccessToken(token); // Recoil 상태에 저장
        setAuthState({ isAuthenticated: true }); 
        navigate('/'); // 메인 페이지로 이동
      } else if (accessToken) {
        console.log('Already logged in with token:', accessToken);
        navigate('/'); // 이미 로그인된 사용자가 있을 경우
      }
    }, [location, setAccessToken, setAuthState, navigate, accessToken]);


    return(
        <h1>로그인 성공! 메인 페이지 이동 중</h1>


    )
        
    
}

export default LoginSuccess;