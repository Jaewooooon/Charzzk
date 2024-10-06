import React, { useState, useEffect } from 'react';
import Modal from 'react-modal';

function PatchModal({ isOpen, onRequestClose, onConfirm, carData }) {
  const [carTypeId, setCarTypeId] = useState('');
  const [number, setNumber] = useState('');
  const [nickname, setNickname] = useState('');

  // carData가 변경되면 상태값을 업데이트
  useEffect(() => {
    if (carData) {
      setCarTypeId(carData.carType?.id || '');
      setNumber(carData?.number || '');
      setNickname(carData?.nickname || '');
    }
  }, [carData]);

  const handleConfirm = () => {
    const updatedCarData = { carTypeId, number, nickname };
    onConfirm(updatedCarData);
  };

  return (
    <Modal isOpen={isOpen} onRequestClose={onRequestClose}>
      <h2>차량 정보 수정</h2>
      <label>차량 번호</label>
      <input
        type="text"
        value={number}
        onChange={(e) => setNumber(e.target.value)}
      />
      <label>닉네임</label>
      <input
        type="text"
        value={nickname}
        onChange={(e) => setNickname(e.target.value)}
      />
      <button onClick={handleConfirm}>확인</button>
      <button onClick={onRequestClose}>취소</button>
    </Modal>
  );
}

export default PatchModal;
