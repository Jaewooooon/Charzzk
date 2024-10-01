import React, { useState, useEffect } from "react";
import axios from 'axios';
import GobackButton from '../../components/GobackButton.jsx';
import '../../styles/SignUp.css';

const SignUp = () => {
    const [carTypes, setCarTypes] = useState([]); // 차량 기종 상태
    const [selectedCarType, setSelectedCarType] = useState(''); // 선택한 차량 기종

    // API 호출 함수
    const fetchCarTypes = async () => {
        try {
            const response = await axios.get('https://j11c208.p.ssafy.io/api/v1/car-types');
            console.log('Fetched car types:', response.data.data); // 응답 데이터 로깅

            // 응답 데이터에서 차량 기종을 추출하여 상태 설정
            if (response.data.code === 200) {
                setCarTypes(response.data.data); // 차량 기종 설정
            } else {
                console.error('Error fetching car types:', response.data.message);
                setCarTypes([]); // 에러 발생 시 빈 배열 설정
            }
        } catch (error) {
            console.error('Error fetching car types:', error);
            setCarTypes([]); // 에러 발생 시 빈 배열 설정
        }
    };

    useEffect(() => {
        fetchCarTypes(); // 컴포넌트 마운트 시 차량 기종 가져오기
    }, []);

    return (
        <>
            <GobackButton />
            <div className='SignUp_contents_box'>
                <div className='SignUp_contents1'>회원정보 및 차량정보를</div>
                <div className='SignUp_contents2'>입력해 주세요.</div>
            </div>

            <div>닉네임</div>
            <input type="text" />
            <button>중복확인</button>

            <div>
                <div>차량 기종</div>
                <select
                    value={selectedCarType}
                    onChange={(e) => setSelectedCarType(e.target.value)} // 선택한 기종을 상태로 저장
                >
                    <option value="">차량 기종을 선택하세요</option>
                    {carTypes.length > 0 ? (
                        carTypes.map((carType) => (
                            <option key={carType.id} value={carType.name}>
                                {carType.name}
                            </option>
                        ))
                    ) : (
                        <option disabled>차량 기종을 불러오는 중입니다...</option>
                    )}
                </select>

                <div>차량 번호</div>
                <input type="text" />

                <div>차량 별명</div>
                <input type="text" /><br />
                <button>회원가입 완료</button>
            </div>
        </>
    );
};

export default SignUp;
