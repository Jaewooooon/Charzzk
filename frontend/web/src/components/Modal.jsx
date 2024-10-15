import React from 'react';
import axios from 'axios'; // axios를 임포트합니다.
import '../styles/Modal.css'; // 스타일을 추가합니다.

const Modal = ({ isOpen, onClose, reportDetails, onConfirm }) => {
  if (!isOpen) return null; // 모달이 열리지 않은 경우에는 아무것도 렌더링하지 않음

  // reportType 값을 한글로 변환하는 함수
  const getReportTypeInKorean = (reportType) => {
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
        return '알 수 없음'; // 예외 처리
    }
  };

  // 확인 완료 버튼 클릭 시 호출되는 함수
  const handleConfirm = async () => {
    if (reportDetails) {
      try {
        const response = await axios.patch(`https://j11c208.p.ssafy.io/api/v1/reports/${reportDetails.id}`, {
          // 필요한 데이터 추가 (예: 확인 상태)
          read: true, // 읽음 상태로 업데이트
        });

        if (response.data.code === 200) {
          console.log('신고가 확인되었습니다.'); // 성공적으로 업데이트됨
          onConfirm(); // 부모 컴포넌트에 상태 업데이트 요청
          onClose(); // 모달 닫기
        } else {
          console.error('Error updating report:', response.data.message);
        }
      } catch (error) {
        console.error('Error updating report:', error);
      }
    }
  };

  return (
    <div className="modal-overlay">
      <div className="modal-content">
        <button className="close-button" onClick={onClose}>X</button>
        {reportDetails && (
          <div>
            <h2>{getReportTypeInKorean(reportDetails.reportType)}</h2>
            <p><strong>작성자(닉네임):</strong> {reportDetails.user.username} ({reportDetails.user.nickname})</p>
            <p><strong>주차장:</strong> {reportDetails.parkingLot.name}</p>
            <p><strong>신고 시각:</strong> {new Date(reportDetails.createdAt).toLocaleString()}</p>
            <div className='reportContents'>            
                <p><strong>내용:</strong> {reportDetails.content}</p>
                {reportDetails.image && <img src={reportDetails.image} alt="신고 이미지" />}
            </div>
            <div className="confirmation-container">
              <button className="ConfirmReport" onClick={handleConfirm}>확인 완료</button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default Modal;
