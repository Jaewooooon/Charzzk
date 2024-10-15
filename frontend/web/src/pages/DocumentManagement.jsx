import { useState, useEffect } from 'react';
import axios from 'axios';
import Modal from '../components/Modal'; // 모달 컴포넌트 임포트
import "../styles/DocumentManagement.css";

const DocumentManagement = () => {
  const [reports, setReports] = useState([]); // 신고 데이터를 저장할 상태 변수
  const [currentPage, setCurrentPage] = useState(1); // 현재 페이지 상태
  const reportsPerPage = 9; // 한 페이지당 보여줄 신고 개수
  const [isModalOpen, setIsModalOpen] = useState(false); // 모달 열림 상태
  const [reportDetails, setReportDetails] = useState(null); // 선택한 신고의 상세 정보

  // 신고 데이터를 가져오는 함수
  const fetchReports = async () => {
    try {
      const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/reports');
      if (response.data.code === 200) {
        // 신고 데이터를 날짜 내림차순으로 정렬 (최신이 맨 위)
        const sortedReports = response.data.data.sort((a, b) => new Date(b.createdAt) - new Date(a.createdAt));
        setReports(sortedReports); // 정렬된 데이터를 상태에 저장
      } else {
        console.error('Error fetching reports:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching reports:', error);
    }
  };

  // 신고 유형 값을 텍스트로 변환하는 함수
  const getReportTypeLabel = (reportType) => {
    switch (reportType) {
      case 'FLIPPED':
        return '로봇이 뒤집힘';
      case 'BROKEN':
        return '로봇 고장';
      case 'DAMAGED':
        return '로봇의 외관 손상';
      case 'NOT_CHARGING':
        return '충전이 안됨';
      case 'ETC':
        return '기타';
      default:
        return '알 수 없음'; // 예상하지 못한 경우 기본값
    }
  };

  // 컴포넌트가 마운트될 때 신고 데이터를 가져옴
  useEffect(() => {
    fetchReports();
  }, []);

  // 페이지 네이션에서 현재 페이지에 해당하는 신고 데이터를 가져오는 함수
  const indexOfLastReport = currentPage * reportsPerPage; // 현재 페이지에서 마지막 신고 데이터 인덱스
  const indexOfFirstReport = indexOfLastReport - reportsPerPage; // 현재 페이지에서 첫 번째 신고 데이터 인덱스
  const currentReports = reports.slice(indexOfFirstReport, indexOfLastReport); // 현재 페이지에서 보여줄 신고 데이터

  // 페이지 번호 클릭 시 호출되는 함수
  const paginate = (pageNumber) => setCurrentPage(pageNumber);

  // 총 페이지 수 계산
  const totalPages = Math.ceil(reports.length / reportsPerPage);

  // 신고 클릭 시 상세 정보를 가져오는 함수
  const handleReportClick = async (reportId) => {
    try {
      const response = await axios.get(`https://j11c208.p.ssafy.io/api/v1/reports/${reportId}`);
      if (response.data.code === 200) {
        setReportDetails(response.data.data); // 신고의 상세 정보 설정
        setIsModalOpen(true); // 모달 열기
      } else {
        console.error('Error fetching report details:', response.data.message);
      }
    } catch (error) {
      console.error('Error fetching report details:', error);
    }
  };

  // 신고 확인 완료 후 데이터를 다시 가져오는 함수
  const handleConfirm = () => {
    fetchReports(); // 신고 데이터를 다시 가져옴
  };

  return (
    <div className='DocumentManagement_container'>
      <div className='IssueList_title'>신고내역</div>
      <div className='IssueList_Box'>
        <div className='IssueList_Contents_title'>
          <div>신고시각</div>
          <div>작성자</div>
          <div>신고 유형</div>
          <div>확인 여부</div>
        </div>

        <div className='IssueList_ContentsBox'>
          {currentReports.length > 0 ? (
            currentReports.map((report) => (
              <div key={report.id} className='IssueList_Contents' onClick={() => handleReportClick(report.id)}>
                <div>{new Date(report.createdAt).toLocaleString()}</div> {/* 신고 시각 */}
                <div>{report.user.username}</div> {/* 작성자 */}
                <div>{getReportTypeLabel(report.reportType)}</div> {/* 신고 유형 변환 */}
                <div>{report.read ? "확인됨" : "미확인"}</div> {/* 확인 여부 */}
              </div>
            ))
          ) : (
            <div className='IssueList_Contents'>신고 내역이 없습니다.</div>
          )}
        </div>

        {/* 페이지 네이션 버튼 */}
        <div className='pagination'>
          {Array.from({ length: totalPages }, (_, i) => (
            <button
              key={i + 1}
              onClick={() => paginate(i + 1)}
              className={currentPage === i + 1 ? 'active' : ''}
            >
              {i + 1}
            </button>
          ))}
        </div>
      </div>

      {/* 모달 컴포넌트 */}
      <Modal 
        isOpen={isModalOpen} 
        onClose={() => setIsModalOpen(false)} 
        reportDetails={reportDetails} 
        onConfirm={handleConfirm} // 확인 완료 시 호출할 함수 전달
      />
    </div>
  );
};

export default DocumentManagement;
