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

          console.log('API Response:', response.data); // API 응답 전체 로그

          if (response.data.code === 200 && response.data.data) {
            if (response.data.data.length === 0) {
              console.log('No car data found (empty array), redirecting to sign-up');
              navigate('/sign-up'); // 차량 데이터가 빈 배열이면 회원가입 페이지로 이동
            } else {
              console.log('Car data exists, redirecting to main page');
              navigate('/'); // 차량 데이터가 있으면 메인 페이지로 이동
            }
          } else {
            console.log('No car data (null or undefined), redirecting to sign-up');
            navigate('/sign-up'); // 차량 데이터가 없으면 회원가입 페이지로 이동
          }
        } catch (error) {
          console.error('Error fetching car data:', error);
          // 에러 발생 시 기본적으로 회원가입 페이지로 이동
          navigate('/sign-up');
        }
      };

      if (token) {
        console.log('Setting access token and auth state');
        setAccessToken(token); // Recoil 상태에 저장
        setAuthState({ isAuthenticated: true });

        // API 호출로 차량 데이터가 있는지 확인
        checkCarData(token);
      } else if (accessToken) {
        console.log('Already logged in with token:', accessToken);
        checkCarData(accessToken); // 이미 로그인된 사용자의 토큰으로 차량 정보 확인
      }
    }, [location, setAccessToken, setAuthState, navigate, accessToken]);

    return (
        <h1>로그인 성공! 페이지 이동 중...</h1>
    );
};

export default LoginSuccess;
