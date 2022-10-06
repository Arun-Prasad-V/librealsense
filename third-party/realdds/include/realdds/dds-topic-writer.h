// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <memory>


namespace eprosima {
namespace fastdds {
namespace dds {
class Publisher;
}  // namespace dds
}  // namespace fastdds
}  // namespace eprosima


namespace realdds {


class dds_topic;
class dds_publisher;


class dds_topic_writer : public eprosima::fastdds::dds::DataWriterListener
{
    std::shared_ptr< dds_topic > const _topic;
    std::shared_ptr< dds_publisher > const _publisher;

    eprosima::fastdds::dds::DataWriter * _writer = nullptr;

public:
    dds_topic_writer( std::shared_ptr< dds_topic > const & topic );
    dds_topic_writer( std::shared_ptr< dds_topic > const & topic, std::shared_ptr< dds_publisher > const & publisher );
    ~dds_topic_writer();

    eprosima::fastdds::dds::DataWriter * get() const { return _writer; }
    eprosima::fastdds::dds::DataWriter * operator->() const { return get(); }

    bool is_running() const { return ( get() != nullptr ); }

    std::shared_ptr< dds_topic > const & topic() const { return _topic; }

    typedef std::function< void( eprosima::fastdds::dds::PublicationMatchedStatus const & ) >
        on_publication_matched_callback;
    void on_publication_matched( on_publication_matched_callback callback )
    {
        _on_publication_matched = std::move( callback );
    }

    class writer_qos : public eprosima::fastdds::dds::DataWriterQos
    {
        using super = eprosima::fastdds::dds::DataWriterQos;

    public:
        writer_qos( eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability
                        = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS,   // default
                    eprosima::fastdds::dds::DurabilityQosPolicyKind durability
                        = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS );  // default is transient local
    };

    // The callbacks should be set before we actually create the underlying DDS objects, so the writer does not
    void run( writer_qos const & = writer_qos() );

    // DataWriterListener
protected:
    // Called when the Publisher is matched (or unmatched) against an endpoint
    void on_publication_matched( eprosima::fastdds::dds::DataWriter * writer,
                                 eprosima::fastdds::dds::PublicationMatchedStatus const & info ) override;

private:
    on_publication_matched_callback _on_publication_matched;
};


}  // namespace realdds
