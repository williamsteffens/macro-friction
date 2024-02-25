#pragma once

#include <QWidget>
#include "Contact.h"

class FrictionLimitsWidget : public QWidget
{
    Q_OBJECT

public:

    FrictionLimitsWidget();

    void setCoeffs(const CoeffsType& _coeffs);

    virtual void paintEvent(QPaintEvent *event) override;

private:

   CoeffsType m_coeffs;

};

