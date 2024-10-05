import React from 'react';
import Modal from 'react-modal';
import DangerConfirm from '../assets/DangerConfirm.png';
import '../components/styles/DeleteModal.css';

const customStyles = {
  overlay: {
    backgroundColor: 'rgba(0, 0, 0, 0.5)', // 반투명 검정 배경
  },
  content: {
    top: '50%',
    left: '50%',
    right: 'auto',
    bottom: 'auto',
    marginRight: '-50%',
    transform: 'translate(-50%, -50%)',
    padding: '25px',
    textAlign: 'center',
    border: 'none', // 모달의 경계선 제거 (선택 사항)
    borderRadius: '10px', // 모달의 모서리를 둥글게 (선택 사항)
    backgroundColor: 'white', // 모달 배경 색상
  },
};


const DeleteModal = ({ isOpen, onRequestClose, onConfirm }) => { // isOpen을 props로 받아야 함
    return (
        <Modal
            isOpen={isOpen}  // props로 전달된 isOpen 사용
            onRequestClose={onRequestClose}
            style={customStyles}
            contentLabel="차량 삭제 확인"
        >
          <img src={DangerConfirm} alt="check" />
            <div className='Delete_Contents'>차량 정보를 삭제하시겠습니까?</div>
            <button onClick={onConfirm} className='DeleteModal_Yes'>예</button>
            <button onClick={onRequestClose} className='DeleteModal_No'>아니오</button>
        </Modal>
    );
};


export default DeleteModal;
