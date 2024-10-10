import React, { useEffect } from 'react'; 
import { useRecoilState, useSetRecoilState } from 'recoil';
import { accessTokenState } from '../../recoil/LoginAtom';
import { useLocation, useNavigate } from 'react-router-dom';
import { authState } from '../../recoil/authState';
import axios from 'axios';

const LoginSuccess = () => {
    const [accessToken, setAccessToken] = useRecoilState(accessTokenState);
    const location = useLocation();
    const navigate = useNavigate();
    const setAuthState = useSetRecoilState(authState);

    useEffect(() => {
      const params = new URLSearchParams(location.search);
      const token = params.get('accessToken');
      
      const checkCarData = async (token) => {
        try {
          const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/cars/me', {
            headers: {
              Authorization: `Bearer ${token}`, // 토큰을 헤더에 포함
            },
          });

          if (response.data.code === 200 && response.data.data) {
            if (response.data.data.length === 0) {
              navigate('/sign-up'); // 차량 데이터가 없으면 회원가입 페이지로 이동
            } else {
              navigate('/'); // 차량 데이터가 있으면 메인 페이지로 이동
            }
          } else {
            navigate('/sign-up'); // 에러 시 회원가입 페이지로 이동
          }
        } catch (error) {
          navigate('/sign-up');
        }
      };

      if (token) {
        setAccessToken(token); // Recoil 상태에 저장
        localStorage.setItem('accessToken', token); // localStorage에 토큰 저장
        setAuthState({ isAuthenticated: true });

        checkCarData(token); // API 호출로 차량 데이터 확인
      } else if (accessToken) {
        checkCarData(accessToken); // 이미 로그인된 사용자의 토큰으로 확인
      }
    }, [location, setAccessToken, setAuthState, navigate, accessToken]);

    return (
        <h1>로그인 성공! 페이지 이동 중...</h1>
    );
};

export default LoginSuccess;
