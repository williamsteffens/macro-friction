#pragma once

#include <QWidget>
#include <QImage>

class FBOWidget : public QWidget
{
    Q_OBJECT

public:

    FBOWidget();

    void setImage(const QImage& _img, int chan = -1);

    virtual void paintEvent(QPaintEvent *event) override;

private:

    QImage m_image;

};
