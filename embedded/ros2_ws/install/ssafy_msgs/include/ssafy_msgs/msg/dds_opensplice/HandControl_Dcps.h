//******************************************************************
// 
//  Generated by IDL to C++ Translator
//  
//  File name: HandControl_Dcps.h
//  Source: ssafy_msgs\msg\HandControl_.idl
//  Generated: timestamp removed to make the build reproducible
//  OpenSplice 6.9.190403OSS
//  
//******************************************************************
#ifndef _HANDCONTROL_DCPS_H_
#define _HANDCONTROL_DCPS_H_

#include "sacpp_mapping.h"
#include "HandControl_.h"
#include "dds_dcps.h"
#include "ssafy_msgs/msg/rosidl_typesupport_opensplice_cpp__visibility_control.h"


namespace ssafy_msgs
{
   namespace msg
   {
      namespace dds_
      {

         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs HandControl_TypeSupportInterface;

         typedef HandControl_TypeSupportInterface * HandControl_TypeSupportInterface_ptr;
         typedef DDS_DCPSInterface_var < HandControl_TypeSupportInterface> HandControl_TypeSupportInterface_var;
         typedef DDS_DCPSInterface_out < HandControl_TypeSupportInterface> HandControl_TypeSupportInterface_out;


         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs HandControl_DataWriter;

         typedef HandControl_DataWriter * HandControl_DataWriter_ptr;
         typedef DDS_DCPSInterface_var < HandControl_DataWriter> HandControl_DataWriter_var;
         typedef DDS_DCPSInterface_out < HandControl_DataWriter> HandControl_DataWriter_out;


         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs HandControl_DataReader;

         typedef HandControl_DataReader * HandControl_DataReader_ptr;
         typedef DDS_DCPSInterface_var < HandControl_DataReader> HandControl_DataReader_var;
         typedef DDS_DCPSInterface_out < HandControl_DataReader> HandControl_DataReader_out;


         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs HandControl_DataReaderView;

         typedef HandControl_DataReaderView * HandControl_DataReaderView_ptr;
         typedef DDS_DCPSInterface_var < HandControl_DataReaderView> HandControl_DataReaderView_var;
         typedef DDS_DCPSInterface_out < HandControl_DataReaderView> HandControl_DataReaderView_out;

         struct HandControl_Seq_uniq_ {};
         typedef DDS_DCPSUFLSeq < HandControl_, struct HandControl_Seq_uniq_> HandControl_Seq;
         typedef DDS_DCPSSequence_var < HandControl_Seq> HandControl_Seq_var;
         typedef DDS_DCPSSequence_out < HandControl_Seq> HandControl_Seq_out;
         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs HandControl_TypeSupportInterface
         :
            virtual public DDS::TypeSupport
         { 
         public:
            typedef HandControl_TypeSupportInterface_ptr _ptr_type;
            typedef HandControl_TypeSupportInterface_var _var_type;

            static HandControl_TypeSupportInterface_ptr _duplicate (HandControl_TypeSupportInterface_ptr obj);
            DDS::Boolean _local_is_a (const char * id);

            static HandControl_TypeSupportInterface_ptr _narrow (DDS::Object_ptr obj);
            static HandControl_TypeSupportInterface_ptr _unchecked_narrow (DDS::Object_ptr obj);
            static HandControl_TypeSupportInterface_ptr _nil () { return 0; }
            static const char * _local_id;
            HandControl_TypeSupportInterface_ptr _this () { return this; }


         protected:
            HandControl_TypeSupportInterface () {};
            ~HandControl_TypeSupportInterface () {};
         private:
            HandControl_TypeSupportInterface (const HandControl_TypeSupportInterface &);
            HandControl_TypeSupportInterface & operator = (const HandControl_TypeSupportInterface &);
         };

         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs HandControl_DataWriter
         :
            virtual public DDS::DataWriter
         { 
         public:
            typedef HandControl_DataWriter_ptr _ptr_type;
            typedef HandControl_DataWriter_var _var_type;

            static HandControl_DataWriter_ptr _duplicate (HandControl_DataWriter_ptr obj);
            DDS::Boolean _local_is_a (const char * id);

            static HandControl_DataWriter_ptr _narrow (DDS::Object_ptr obj);
            static HandControl_DataWriter_ptr _unchecked_narrow (DDS::Object_ptr obj);
            static HandControl_DataWriter_ptr _nil () { return 0; }
            static const char * _local_id;
            HandControl_DataWriter_ptr _this () { return this; }

            virtual DDS::LongLong register_instance (const HandControl_& instance_data) = 0;
            virtual DDS::LongLong register_instance_w_timestamp (const HandControl_& instance_data, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long unregister_instance (const HandControl_& instance_data, DDS::LongLong handle) = 0;
            virtual DDS::Long unregister_instance_w_timestamp (const HandControl_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long write (const HandControl_& instance_data, DDS::LongLong handle) = 0;
            virtual DDS::Long write_w_timestamp (const HandControl_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long dispose (const HandControl_& instance_data, DDS::LongLong handle) = 0;
            virtual DDS::Long dispose_w_timestamp (const HandControl_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long writedispose (const HandControl_& instance_data, DDS::LongLong handle) = 0;
            virtual DDS::Long writedispose_w_timestamp (const HandControl_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long get_key_value (HandControl_& key_holder, DDS::LongLong handle) = 0;
            virtual DDS::LongLong lookup_instance (const HandControl_& instance_data) = 0;

         protected:
            HandControl_DataWriter () {};
            ~HandControl_DataWriter () {};
         private:
            HandControl_DataWriter (const HandControl_DataWriter &);
            HandControl_DataWriter & operator = (const HandControl_DataWriter &);
         };

         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs HandControl_DataReader
         :
            virtual public DDS::DataReader
         { 
         public:
            typedef HandControl_DataReader_ptr _ptr_type;
            typedef HandControl_DataReader_var _var_type;

            static HandControl_DataReader_ptr _duplicate (HandControl_DataReader_ptr obj);
            DDS::Boolean _local_is_a (const char * id);

            static HandControl_DataReader_ptr _narrow (DDS::Object_ptr obj);
            static HandControl_DataReader_ptr _unchecked_narrow (DDS::Object_ptr obj);
            static HandControl_DataReader_ptr _nil () { return 0; }
            static const char * _local_id;
            HandControl_DataReader_ptr _this () { return this; }

            virtual DDS::Long read (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_w_condition (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long take_w_condition (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long read_next_sample (HandControl_& received_data, DDS::SampleInfo& sample_info) = 0;
            virtual DDS::Long take_next_sample (HandControl_& received_data, DDS::SampleInfo& sample_info) = 0;
            virtual DDS::Long read_instance (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take_instance (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_next_instance (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take_next_instance (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_next_instance_w_condition (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long take_next_instance_w_condition (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long return_loan (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq) = 0;
            virtual DDS::Long get_key_value (HandControl_& key_holder, DDS::LongLong handle) = 0;
            virtual DDS::LongLong lookup_instance (const HandControl_& instance) = 0;

         protected:
            HandControl_DataReader () {};
            ~HandControl_DataReader () {};
         private:
            HandControl_DataReader (const HandControl_DataReader &);
            HandControl_DataReader & operator = (const HandControl_DataReader &);
         };

         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs HandControl_DataReaderView
         :
            virtual public DDS::DataReaderView
         { 
         public:
            typedef HandControl_DataReaderView_ptr _ptr_type;
            typedef HandControl_DataReaderView_var _var_type;

            static HandControl_DataReaderView_ptr _duplicate (HandControl_DataReaderView_ptr obj);
            DDS::Boolean _local_is_a (const char * id);

            static HandControl_DataReaderView_ptr _narrow (DDS::Object_ptr obj);
            static HandControl_DataReaderView_ptr _unchecked_narrow (DDS::Object_ptr obj);
            static HandControl_DataReaderView_ptr _nil () { return 0; }
            static const char * _local_id;
            HandControl_DataReaderView_ptr _this () { return this; }

            virtual DDS::Long read (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_w_condition (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long take_w_condition (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long read_next_sample (HandControl_& received_data, DDS::SampleInfo& sample_info) = 0;
            virtual DDS::Long take_next_sample (HandControl_& received_data, DDS::SampleInfo& sample_info) = 0;
            virtual DDS::Long read_instance (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take_instance (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_next_instance (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take_next_instance (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_next_instance_w_condition (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long take_next_instance_w_condition (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long return_loan (HandControl_Seq& received_data, DDS::SampleInfoSeq& info_seq) = 0;
            virtual DDS::Long get_key_value (HandControl_& key_holder, DDS::LongLong handle) = 0;
            virtual DDS::LongLong lookup_instance (const HandControl_& instance) = 0;

         protected:
            HandControl_DataReaderView () {};
            ~HandControl_DataReaderView () {};
         private:
            HandControl_DataReaderView (const HandControl_DataReaderView &);
            HandControl_DataReaderView & operator = (const HandControl_DataReaderView &);
         };

      }
   }
}




#endif
