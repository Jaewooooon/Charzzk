import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import './App.css';
import MainPage from './pages/MainPage';
import ChargeMap from './pages/ChargeMap';
import ChargeStart from './pages/ChargeStart';
import ChargeStatus from './pages/ChargeStatus';
import CarManagement from './pages/Mypage/CarManagement';
import PaymentManagement from './pages/Mypage/PaymentManagement';
import ReportIssue from './pages/Mypage/ReportIssue';

function App() {
  return (
    <Router>
      <Routes>
        {/* 메인 페이지 */}
        <Route path="/" element={<MainPage />} />

        {/* 충전 지도 페이지 */}
        <Route path="/charge-map" element={<ChargeMap />} />

        {/* 충전 시작 페이지 */}
        <Route path="/charge-start" element={<ChargeStart />} />

        {/* 충전 상태 페이지 */}
        <Route path="/charge-status" element={<ChargeStatus />} />

        {/* 마이페이지 - 차량 관리 */}
        <Route path="/mypage/car-management" element={<CarManagement />} />

        {/* 마이페이지 - 결제 수단 관리 */}
        <Route path="/mypage/payment-management" element={<PaymentManagement />} />

        {/* 마이페이지 - 문제 신고 */}
        <Route path="/mypage/report-issue" element={<ReportIssue />} />
      </Routes>
    </Router>
  );
}

export default App;
