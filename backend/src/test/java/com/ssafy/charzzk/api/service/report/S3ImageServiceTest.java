package com.ssafy.charzzk.api.service.report;

import com.amazonaws.services.s3.AmazonS3;
import com.amazonaws.services.s3.model.PutObjectRequest;
import com.ssafy.charzzk.core.exception.BaseException;
import com.ssafy.charzzk.core.exception.ErrorCode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.springframework.mock.web.MockMultipartFile;
import org.springframework.test.context.ActiveProfiles;
import org.springframework.test.util.ReflectionTestUtils;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.net.URL;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyString;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.mock;

@ExtendWith(MockitoExtension.class)
@Transactional
@ActiveProfiles("test")
class S3ImageServiceTest {

    @Mock
    private AmazonS3 amazonS3;

    @InjectMocks
    private S3ImageService s3ImageService;

    @BeforeEach
    void setUp() {
        ReflectionTestUtils.setField(s3ImageService, "bucketName", "test-bucket");
    }

    @DisplayName("정상적인 이미지 업로드 요청이 오면 이미지 업로드가 성공적으로 수행된다.")
    @Test
    void uploadImage() throws Exception {
        // given
        String originalFilename = "test-image.jpg";
        MockMultipartFile mockMultipartFile = new MockMultipartFile("image", originalFilename, "image/jpeg", "test image content".getBytes());

        given(amazonS3.putObject(any(PutObjectRequest.class))).willReturn(null);
        given(amazonS3.getUrl(anyString(), anyString()))
                .willReturn(new URL("https://mockurl.com/test-image.jpg"));

        // when
        String result = s3ImageService.upload(mockMultipartFile);

        // then
        assertThat(result).isEqualTo("https://mockurl.com/test-image.jpg");
    }


    @DisplayName("빈 파일을 업로드하면 예외가 발생한다.")
    @Test
    public void uploadWithEmptyFile() {
        // given
        MultipartFile emptyFile = new MockMultipartFile("image", "", "image/png", new byte[0]);

        // when & then
        BaseException exception = assertThrows(BaseException.class, () -> s3ImageService.upload(emptyFile));
        assertThat(exception.getErrorCode()).isEqualTo(ErrorCode.EMPTY_FILE_EXCEPTION);
    }

    @DisplayName("허용되지 않 파일 확장자를 업로드하면 예외가 발생한다.")
    @Test
    public void uploadWithInvalidFileExtension() {
        // given
        MultipartFile invalidFile = new MockMultipartFile("image", "invalid-file.txt", "text/plain", "content".getBytes());

        // when & then
        BaseException exception = assertThrows(BaseException.class, () -> s3ImageService.upload(invalidFile));
        assertThat(exception.getErrorCode()).isEqualTo(ErrorCode.INVALID_FILE_EXTENSION);
    }

    @DisplayName("파일 크기가 5MB를 초과하면 예외가 발생한다.")
    @Test
    public void uploadFileWithExceedingSizeLimit() {
        // given
        byte[] largeContent = new byte[5_242_881]; // 5MB + 1 byte
        MultipartFile largeFile = new MockMultipartFile("image", "large-image.png", "image/png", largeContent);

        // when & then
        BaseException exception = assertThrows(BaseException.class, () -> s3ImageService.upload(largeFile));
        assertThat(exception.getErrorCode()).isEqualTo(ErrorCode.FILE_SIZE_EXCEEDS_LIMIT);
    }

    @DisplayName("S3 업로드 중 IO 예외가 발생하면 예외가 발생한다.")
    @Test
    public void uploadImageWithIoException() throws IOException {
        // given
        String originalFilename = "test-image.png";
        byte[] content = "image-content".getBytes();
        MultipartFile multipartFile = mock(MultipartFile.class);

        given(multipartFile.getOriginalFilename()).willReturn(originalFilename);
        given(multipartFile.getInputStream()).willThrow(new IOException("Test IO Exception"));

        // when & then
        BaseException exception = assertThrows(BaseException.class, () -> s3ImageService.upload(multipartFile));
        assertThat(exception.getErrorCode()).isEqualTo(ErrorCode.IO_EXCEPTION_ON_IMAGE_UPLOAD);
    }


}