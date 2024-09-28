import React, { useEffect } from 'react'; 
import { useRecoilState } from 'recoil';
import { accessTokenState } from '../../recoil/LoginAtom';
import { useLocation, useNavigate } from 'react-router-dom';
import { authState } from '../../recoil/authState';

const LoginSuccess = () => {
    const [accessToken, setAccessToken] = useRecoilState(accessTokenState);
    const location = useLocation();
    const navigate = useNavigate();
    const setAuthState = useSetRecoilState(authState);

    useEffect(() => {
        // URL에서 accessToken 추출
        const params = new URLSearchParams(location.search);
        const token = params.get('accessToken');
        console.log(token);
        
        if (token) {
          setAccessToken(token); // Recoil 상태에 저장
          setAuthState(true);
          navigate('/'); // 로그인 후 대시보드 페이지로 이동
        } else if (accessToken) {
          // 이미 로그인된 사용자가 있을 경우
          navigate('/');
        }
      }, [setAuthState,location, setAccessToken, navigate, accessToken]);


    return(
        <h1>로그인 성공! 메인 페이지 이동 중</h1>


    )
        
    
}

export default LoginSuccess;