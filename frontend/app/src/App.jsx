import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import './App.css';
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
  return (
    <RecoilRoot>
    <Router>
    <Routes>

{/* 메인 페이지 */}
<Route path="/" element={<PrivateRoute element={<MainPage />} />}/>

{/* 충전 지도 페이지 */}
<Route path="/charge-map" element={<PrivateRoute element={<ChargeMap />} />} />

{/* 충전 시작 페이지 */}
<Route path="/charge-start" element={<PrivateRoute element={<ChargeStart />} />} />

{/* 충전 상태 페이지 */}
<Route path="/charge-status" element={<PrivateRoute element={<ChargeStatus />} />} />

{/* 마이페이지 - 차량 관리 */}
<Route path="/mypage/car-management" element={<PrivateRoute element={<CarManagement />} />} />

{/* 마이페이지 - 결제 수단 관리 */}
<Route path="/mypage/user-management" element={<PrivateRoute element={<UserManagement />} />} />

{/* 마이페이지 - 문제 신고 */}
<Route path="/mypage/report-issue" element={<PrivateRoute element={<ReportIssue />} />} />

{/*로그인 페이지*/}
<Route path="/login" element={<Login />} />

{/* 로그인 성공 페이지 */}
<Route path="/auth/callback" element={<LoginSuccess />}  />

{/* 회원가입 페이지 */}
<Route path="/sign-up" element={<PrivateRoute element={<SignUp />} />} />

</Routes>
    </Router>
    </RecoilRoot>
  );
}

export default App;
