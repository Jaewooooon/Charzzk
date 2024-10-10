import React, { useEffect } from 'react'; 
import { useSetRecoilState } from 'recoil';
import { accessTokenState } from './recoil/LoginAtom';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import MainPage from './pages/MainPage';
import ChargeMap from './pages/ChargeMap';
import ChargeStart from './pages/ChargeStart';
import ChargeStatus from './pages/ChargeStatus';
import CarManagement from './pages/Mypage/CarManagement';
import UserManagement from './pages/Mypage/UserManagement';
import ReportIssue from './pages/Mypage/ReportIssue';
import LoginSuccess from './pages/Login/LoginSuccess';
import Login from './pages/Login/Login';
import PrivateRoute from './components/PrivateRoute';
import SignUp from './pages/Login/SignUp';
import { RecoilRoot } from 'recoil';

function App() {
  const setAccessToken = useSetRecoilState(accessTokenState);

  useEffect(() => {
    // localStorage에서 토큰을 가져와 Recoil 상태에 복원
    const token = localStorage.getItem('accessToken');
    if (token) {
      setAccessToken(token);
    }
  }, [setAccessToken]);

  return (
    <RecoilRoot>
      <Router>
        <Routes>
          <Route path="/" element={<PrivateRoute element={<MainPage />} />} />
          <Route path="/charge-map" element={<PrivateRoute element={<ChargeMap />} />} />
          <Route path="/charge-start" element={<PrivateRoute element={<ChargeStart />} />} />
          <Route path="/charge-status" element={<PrivateRoute element={<ChargeStatus />} />} />
          <Route path="/mypage/car-management" element={<PrivateRoute element={<CarManagement />} />} />
          <Route path="/mypage/user-management" element={<PrivateRoute element={<UserManagement />} />} />
          <Route path="/mypage/report-issue" element={<PrivateRoute element={<ReportIssue />} />} />
          <Route path="/login" element={<Login />} />
          <Route path="/auth/callback" element={<LoginSuccess />} />
          <Route path="/sign-up" element={<PrivateRoute element={<SignUp />} />} />
        </Routes>
      </Router>
    </RecoilRoot>
  );
}

export default App;
